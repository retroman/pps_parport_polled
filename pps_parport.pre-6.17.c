/*
 * pps_parport.c -- kernel parallel port PPS client
 *
 *
 * Copyright (C) 2009   Alexander Gordeev <lasaine@lvk.cs.msu.su>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/parport.h>
#include <linux/pps_kernel.h>
#include <linux/workqueue.h>
#include <linux/timekeeping.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/sched/clock.h>

#define DRVDESC "parallel port PPS client"

/* module parameters */
#define PM_SHUTDOWN -1
#define PM_INT  0
#define PM_POLL 1
#define PM_NONE 2
static int ppps_mode[PARPORT_MAX+1] = {[0 ... PARPORT_MAX] = 0};
module_param_array(ppps_mode, int, NULL, 0);
MODULE_PARM_DESC(ppps_mode,
	"each parallel port can have a different mode: "
	"0 for interrupt-only timestamp capture (default), "
	"1 for polling mode timestamp capture -- "
	"requires high-resoultion timers (kernel CONFIG_HIGH_RES_TIMERS=y), "
	"2 to ignore port");

#define CLEAR_WAIT_MAX 1000
static unsigned int clear_wait[PARPORT_MAX+1] = {[0 ... PARPORT_MAX] = 100};
module_param_array(clear_wait, uint, NULL, 0);
MODULE_PARM_DESC(clear_wait,
	"per-port: "
	"Maximum number of port reads when polling for signal 'clear' edge, "
	"0-1000, default 100, 0 disables. NOTE: Many GPS device manuals "
	"state that their clear edges are not to be used for timing.");

/*
 * read_status and write_data don't get inlined due to no-strict-aliasing,
 * and won't get inlined when built as a module even with -fstrict-aliasing and
 * restrict pointers, so use inb() and outb(). echo latency is reduced
 * significantly even on fast machines
 */
static bool ppps_direct = false;
DEFINE_STATIC_KEY_FALSE(direct);
module_param(ppps_direct, bool, 0);
MODULE_PARM_DESC(ppps_direct,
	"for all ports: "
	"Use direct IO, default off. Works only with PC-style hardware "
	"(chips that use the 'parport_pc' driver)");

static bool ppps_echo = false;
DEFINE_STATIC_KEY_FALSE(echo);
module_param(ppps_echo, bool, 0);
MODULE_PARM_DESC(ppps_echo, 
	"for all ports: echo ACK to data lines, "
	"true to enable, false to disable. default false");

static unsigned int ppps_interval[PARPORT_MAX+1] = {[0 ... PARPORT_MAX] = 1000};
module_param_array(ppps_interval, uint, NULL, 0);
MODULE_PARM_DESC(ppps_interval,
	"per-port: expected/generated PPS interval "
	"in milliseconds, 500-10000, default 1000ms");

static unsigned int ppps_mask[PARPORT_MAX+1] = {[0 ... PARPORT_MAX] = 0};
module_param_array(ppps_mask, uint, NULL, 0);
MODULE_PARM_DESC(ppps_mask,
	"per-port, interrupt mode only: "
	"pulse mask time in milliseconds, 0 or 100-10000, "
	"default 0 (disabled). Used to prevent recapture on level-triggered "
	"hardware. Activates only when falling edge capture is disabled.");

static bool ppps_out_ts_clear_edge[PARPORT_MAX+1] = 
	{[0 ... PARPORT_MAX] = false};
module_param_array(ppps_out_ts_clear_edge, bool, NULL, false);
MODULE_PARM_DESC(ppps_out_ts_clear_edge,
	"per-port: edge to take timestamp on, "
	"true for clear edge, false for assert edge. default false");

/*
 * parallel ports are supposed to trigger an interrupt on the low-to-high
 * edge, but some are high-to-low
 */
#ifndef ACK_IS_HW_INVERTED
#define ASSERTED  != 0
#define CLEARED   == 0
#else
#define ASSERTED  == 0
#define CLEARED   != 0
#endif

#define CLEAR_WAIT_MAX_ERRORS 5

#define ECHO_DATA_HI 0xff
#define ECHO_DATA_LO 0x00

#define STARTUP_DELAY msecs_to_jiffies(2200)

/*
 * time to add or subtract from the hrt wait interval if it is
 * not on target
 */
#define ADJUST_NS 800
#define ADJUST_DITHER 200

/*
 * on the first polling loop try, start polling before the expected signal
 * arrival time to accomidate hrtimer wake-up variation and clock stepping
 */
#define POLL_START_US 250

/*
 * An IO read may take anywhere from ~200ns (PCI with fast DEVSEL
 * using back-to-back)
 * to ~1700ns (single-lane PCIe) on a fast machine.
 */
#define LOOP_CAL_COUNT 10000
#define MIN_TRYS 20

static DEFINE_IDA(pps_client_index);

/* internal per-port structure */
struct pps_client_pp {
	int      interrupt_disabled; /*bool, but use int for alignment*/
	int      mode;
	int      status_addr;
	int      data_addr;
	struct   pardevice * __restrict pardev;
	/* cache doesnt matter for the rest */
	unsigned int cw;
	unsigned int interval;
	int      poll_trys;
	unsigned int cw_err;
	unsigned int d_dir;
	int      poll_target;
	int      prev_pollct;
	ktime_t  hrt_period;
	struct   hrtimer hrt;
	unsigned long hrt_period_ns;
	ktime_t  hrt_initial_period;
	unsigned int mask_jiffies;
	struct   workqueue_struct * __restrict wq;
	struct   delayed_work dw;
	struct   pps_device * __restrict pps;
	int  index;	/*device_number*/
};

/* do the conversion separately to reduce echo latency */
static void ss_pps_event(struct pps_device * __restrict pps, struct system_time_snapshot * __restrict snap, int event)
{
	struct pps_event_time ts;
	ts.ts_real = ktime_to_timespec64(snap->real);
#ifdef CONFIG_NTP_PPS
	ts.ts_raw = ktime_to_timespec64(snap->raw);
#endif
	pps_event(pps, &ts, event, NULL);
}

/* delayed-work handler */
static void pps_work(struct work_struct * __restrict const work)
{
	struct pps_client_pp * __restrict const dev = 
		container_of(work, struct pps_client_pp, dw.work);
	struct parport * __restrict const port = dev->pardev->port;

	port->ops->enable_irq(port);
	dev->interrupt_disabled = 0;
}

/* hrtimer poll hander */
__aligned(64) static enum hrtimer_restart pps_hrt(struct hrtimer * __restrict const timer)
{
	struct ____cacheline_aligned system_time_snapshot ss_assert, ss_clear;
	struct pps_client_pp * __restrict const dev = 
		container_of(timer, struct pps_client_pp, hrt);
	unsigned long flags;

	/* local copies to minimize latency if strict aliasing is not enabled */
	const unsigned int max_polls = dev->poll_trys;
	const unsigned int s_addr = dev->status_addr, d_addr = dev->data_addr;
	unsigned int p, a_polls, cw_trys = dev->cw;
	struct parport * __restrict const port = dev->pardev->port;
	bool poll_fail;
	unsigned char (* const read_status)(struct parport *) = port->ops->read_status;
	void (* const write_data)(struct parport *, unsigned char value) = port->ops->write_data;
	int adj;

	if (dev->mode == PM_SHUTDOWN) {
		return HRTIMER_NORESTART; /* shutting down */
	}

	/* prepare rising edge echo if falling edge capture disabled */
	if (ppps_echo && cw_trys == 0) {
		if (static_branch_likely(&direct))
			outb(ECHO_DATA_LO, d_addr);
		else
			write_data(port, ECHO_DATA_LO);
	}

	/* wait for input rising edge */
	local_irq_save(flags);
	sched_clock_idle_sleep_event();

	p = max_polls;
	ktime_get_snapshot(&ss_assert); /* warm up cache */
	if (static_branch_likely(&direct)) {
		while (p-- && (inb(s_addr) &
			PARPORT_STATUS_ACK) CLEARED){}
		ktime_get_snapshot(&ss_assert);
	} else {
		while (p-- && (read_status(port) & 
			PARPORT_STATUS_ACK) CLEARED){}
		ktime_get_snapshot(&ss_assert); /* duplicate to prevent jump latency in case no static key support */
	}
	a_polls = max_polls - p;
	if (ppps_echo && a_polls > 1 && a_polls < max_polls) {
		if (static_branch_likely(&direct))
			outb(ECHO_DATA_HI, d_addr);
		else
			write_data(port, ECHO_DATA_HI);
	}
	if (cw_trys > 0 && a_polls > 1 && a_polls < max_polls) {
		/* NOTE: echo will be lowered even if there was a timeout */
		ktime_get_snapshot(&ss_clear);
		if (static_branch_likely(&direct)) {
			while (cw_trys-- && (inb(s_addr) &
				PARPORT_STATUS_ACK) ASSERTED){}
			ktime_get_snapshot(&ss_clear);
			if (static_branch_likely(&echo))
				outb(ECHO_DATA_LO, d_addr);
		} else {
			while (cw_trys-- && (read_status(port) &
				PARPORT_STATUS_ACK) ASSERTED){}
			ktime_get_snapshot(&ss_clear);
			if (static_branch_likely(&echo))
				write_data(port, ECHO_DATA_LO);
		}

		local_irq_restore(flags);
		if (unlikely(cw_trys == 0)) {
			local_irq_restore(flags);
			if (dev->cw_err++ >= CLEAR_WAIT_MAX_ERRORS) {
				dev->cw = 0;
				dev_err(dev->pps->dev, "disabled clear edge capture after %d consecutive timeouts\n",
					CLEAR_WAIT_MAX_ERRORS);
			}
		} else {
			dev->cw_err = 0;
		}
	} else {
		local_irq_restore(flags);
	}

	if (unlikely(a_polls == 1)) {
		poll_fail = true;
		dev_info(dev->pps->dev,
			"ACK asserted at poll start, disabling polling until next interrupt\n");
	} else if (unlikely(a_polls >= max_polls)) {
		poll_fail = true;
		dev_info(dev->pps->dev,
			"poll timeout, disabling polling until next interrupt\n");
	} else {
		poll_fail = false;
		ss_pps_event(dev->pps, &ss_assert, PPS_CAPTUREASSERT);
		if (dev->cw > 0)
			ss_pps_event(dev->pps, &ss_clear, PPS_CAPTURECLEAR);
	}

	if (unlikely(poll_fail)) {
		/* reset to standard interval */
		dev->hrt_period = ns_to_ktime(dev->hrt_period_ns);
		dev->prev_pollct = dev->poll_target;

		/*
		 * re-enable interrupt with a delay long enough to prevent
		 * triggering the interrupt handler with the current pulse
		 */
		queue_delayed_work(dev->wq, &dev->dw, dev->mask_jiffies);

		return HRTIMER_NORESTART;
	}

	/* adjust wait time towards the poll count target */
	if (a_polls < dev->poll_target && a_polls <= dev->prev_pollct) {
		adj = -ADJUST_NS;
	} else if (a_polls > dev->poll_target && a_polls >= dev->prev_pollct) {
		adj = ADJUST_NS;
	} else {
		if (dev->d_dir++ % 1)
			adj = -ADJUST_DITHER;
		else
			adj = +ADJUST_DITHER;
	}
	if (adj >= 0) {
		dev->hrt_period = ktime_add_ns(dev->hrt_period, adj);
		dev_dbg(dev->pps->dev, "polled ok: %d  wait time  added %llu\n",
			a_polls, ktime_to_ns(dev->hrt_period));
	} else {
		dev->hrt_period = ktime_sub_ns(dev->hrt_period, -adj);
		dev_dbg(dev->pps->dev, "polled ok: %d  wait time subbed %llu\n",
			a_polls, ktime_to_ns(dev->hrt_period));
	}
	dev->prev_pollct = a_polls;

	hrtimer_forward_now(timer, dev->hrt_period);
	return HRTIMER_RESTART;
}

/* parport interrupt handler */
__aligned(64) static void parport_irq(void * __restrict const handle)
{
	unsigned int cw_trys, s_addr, d_addr;
	struct pps_client_pp * __restrict dev;
	struct ____cacheline_aligned system_time_snapshot ss_assert, ss_clear;
	struct parport * __restrict port;
	unsigned char (*read_status)(struct parport *);
	void (*write_data)(struct parport *, unsigned char value);

	/* 
	 * note: don't need local_irq_save() since 2.6.35
	 * get the timestamp here even in polling mode. will waste CPU
	 * in polled mode, and if using a shared interrupt, but reduces latency
	 */
	ktime_get_snapshot(&ss_assert);

	dev = handle;
	if (unlikely(dev->interrupt_disabled)) {
		goto ignore_irq; /* not our interrupt */
	}

	if (dev->mode == PM_INT) {
		/* echo here before reading port, but may cause false echos */
		if (static_branch_likely(&echo)) {
			if (static_branch_likely(&direct)) {
				outb(ECHO_DATA_HI, dev->data_addr);
			} else {
				port = dev->pardev->port;
				port->ops->write_data(port, ECHO_DATA_HI);
			}
		}
	}

	port = dev->pardev->port;
	if (unlikely((port->ops->read_status(port) 
		& PARPORT_STATUS_ACK) CLEARED))
		goto ignore_irq_report;

	if (dev->mode == PM_POLL) {
		/* mask interrupt. hrtimer will re-enable if needed */
		port->ops->disable_irq(port);
		dev->interrupt_disabled = 1;
		/*
		 * Polling isn't done until one interrupt is handled.
		 * also: hrtimer may have started polling too late,
		 * or PPS input stopped and then restarted,
		 * or system time was stepped
		 */
		hrtimer_start(&dev->hrt, dev->hrt_initial_period,
			HRTIMER_MODE_REL);
	} else if (dev->cw) {
		/* NOTE: echo will be lowered even if there was a timeout */
		cw_trys = dev->cw;
		if (static_branch_likely(&direct)) {
			s_addr = dev->status_addr;
			d_addr = dev->data_addr;
			ktime_get_snapshot(&ss_clear);
			while (cw_trys-- && (inb(s_addr) &
				PARPORT_STATUS_ACK) ASSERTED){}
			ktime_get_snapshot(&ss_clear);
			if (static_branch_likely(&echo))
				outb(ECHO_DATA_LO, d_addr);
		} else {
			read_status = port->ops->read_status;
			write_data = port->ops->write_data;
			ktime_get_snapshot(&ss_clear);
			while (cw_trys-- && (read_status(port) &
				PARPORT_STATUS_ACK) ASSERTED){}
			ktime_get_snapshot(&ss_clear);
			if (static_branch_likely(&echo))
				write_data(port, ECHO_DATA_LO);
		}
		ss_pps_event(dev->pps, &ss_assert, PPS_CAPTUREASSERT);

		if (unlikely(cw_trys == 0)) {
			if (dev->cw_err++ >= CLEAR_WAIT_MAX_ERRORS) {
				dev->cw = 0;
				dev_err(dev->pps->dev, "disabled clear edge capture after %d consecutive timeouts\n",
					CLEAR_WAIT_MAX_ERRORS);
			}
		} else {
			ss_pps_event(dev->pps, &ss_clear, PPS_CAPTURECLEAR);
			dev->cw_err = 0;
		}
	} else {
		ss_pps_event(dev->pps, &ss_assert, PPS_CAPTUREASSERT);
		/*
		 * level-triggered interrupt workaround: wait before
		 * re-enabling interrupt. The interrupt must be disabled for
		 * the entire duration that the pulse is asserted.
		 */
		if (dev->mask_jiffies > 0) {
			port->ops->disable_irq(port);
			dev->interrupt_disabled = 1;
			queue_delayed_work(dev->wq, &dev->dw,
				dev->mask_jiffies);
		}
		/* lower echo to create next echo rising edge */
		if (ppps_echo)
			port->ops->write_data(port, ECHO_DATA_LO);
	}
	return;

ignore_irq_report:
	dev_err(dev->pps->dev, "lost the signal, or got a shared interrupt from another device\n");
	if (ppps_echo) {
		port = dev->pardev->port;
		port->ops->write_data(port, ECHO_DATA_LO);
	}
ignore_irq:
	return;
}

static void parport_attach(struct parport * __restrict const port)
{
	struct pardev_cb pps_client_cb;
	int index;
	int pollct, ns_perpoll, s_addr, d_addr, port_num;
	struct timespec64 start, end, len;
	s64 len_ns, start_ns, end_ns;
	unsigned long flags;
	unsigned char (*read_status)(struct parport *);
	void (*write_data)(struct parport *, unsigned char value);
	unsigned char sta;

	struct pps_client_pp * __restrict device;
	struct pps_source_info info = {
		.name		= KBUILD_MODNAME,
		.path		= "",
		/*
		 * Our echo is private to avoid latency from going through
		 * the API. Maybe add an echo on a data pin if a user of the
		 * API wants to see how much latency it adds.
		 */
		.echo           = NULL,
		.mode		= PPS_CAPTUREBOTH | \
				  PPS_OFFSETASSERT | PPS_OFFSETCLEAR | \
				  PPS_CANWAIT | PPS_TSFMT_TSPEC,
		.owner		= THIS_MODULE,
		.dev		= NULL
	};

	device = kzalloc(sizeof(struct pps_client_pp), GFP_KERNEL);
	if (!device) {
		pr_err("memory allocation failed, not attaching to %s\n",
			port->name);
		return;
	}

 	index = ida_simple_get(&pps_client_index, 0, 0, GFP_KERNEL);
 	memset(&pps_client_cb, 0, sizeof(pps_client_cb));
 	pps_client_cb.private = device;
 	pps_client_cb.irq_func = parport_irq;
 	pps_client_cb.flags = PARPORT_FLAG_EXCL;
 	device->pardev = parport_register_dev_model(port,
		KBUILD_MODNAME,
	        &pps_client_cb,
	        index);
	if (!device->pardev) {
		pr_err("couldn't register with %s\n", port->name);
		goto err_free;
	}

	if (parport_claim_or_block(device->pardev) < 0) {
		pr_err("couldn't claim %s\n", port->name);
		goto err_unregister_dev;
	}
	if (device->pardev->port->irq == PARPORT_IRQ_NONE) {
		pr_err("%s reports no interrupt available\n", port->name);
		goto err_release_dev;
	}
	device->pps = pps_register_source(&info,
		PPS_CAPTUREBOTH | PPS_OFFSETASSERT | PPS_OFFSETCLEAR );
	if (IS_ERR(device->pps)) {
		pr_err("%s couldn't register PPS source\n", port->name);
		goto err_release_dev;
	}

	if (port->base_hi)
		pr_info("pps%d attached to %s at 0x%lx & 0x%lx\n",
			device->pps->id, port->name,
			port->base, port->base_hi);
	else
		pr_info("pps%d attached to %s at 0x%lx\n",
			device->pps->id, port->name, port->base);

	if (ppps_direct && ((port->modes & PARPORT_MODE_PCSPP) == 0))
		pr_info("NOTE: ppps_direct enabled, but %s PARPORT_MODE_PCSPP bit is not set\n",
		        port->name);

	/* check per-port parameters specified at module load time*/
	port_num = -1;
	sta = kstrtoint(&port->name[strcspn(port->name, "0123456789")], 10, &port_num);

	if (0 != sta || port_num < 0 || port_num > PARPORT_MAX) {
		pr_err("couldn't get port number from %s\n", port->name);
		goto err_release_source;
	}

	device->mode = ppps_mode[port_num];
	if (device->mode < 0 || device->mode > PM_NONE) {
		pr_err("ppps_mode of %d for %s is invalid, must be 0 to 2\n",
			device->mode, port->name);
		goto err_release_source;
	}
	if (device->mode == PM_NONE) {
		pr_info("skipping %s\n", port->name);
		goto err_release_source;
	}
	device->interval = ppps_interval[port_num];
	if (device->interval > 10000 || device->interval < 500) {
		pr_err("Expected PPS interval of %d for %s is out of range. must be 500 to 10000 (milliseconds)\n",
			device->interval, port->name);
		goto err_release_source;
	}

	device->cw = clear_wait[port_num];
	if (device->cw > CLEAR_WAIT_MAX) {
		pr_err("clear_wait value of %d polls for %s is out of range. must be 0 to %d\n",
			device->cw, port->name, CLEAR_WAIT_MAX);
		goto err_release_source;
	} else if (clear_wait == 0) {
		pr_info("clear edge capture disabled\n");
	}

	if (ppps_mask[port_num] != 0 &&
	    (ppps_mask[port_num] > 10000 || ppps_mask[port_num] < 100)) {
		pr_err("Interrupt mask value of %d for %s is out of range. must be 0 or 100 to 10000 (milliseconds)\n",
			ppps_mask[port_num], port->name);
		goto err_release_source;
	}

	/* start with interrupt pin masked */
	port->ops->disable_irq(port);
	device->interrupt_disabled = 1;

	device->cw_err = 0;
	device->data_addr = port->base;
	device->status_addr = port->base + 1;
	s_addr = device->status_addr;
	d_addr = device->data_addr;

	if (device->mode == PM_POLL) {
		if (KTIME_MONOTONIC_RES != KTIME_HIGH_RES) {
			pr_err("polling mode requested, but hrtimer_resolution didn't report KTIME_HIGH_RES\n");
			goto err_release_source;
		}
		pr_info("using hrtimer polling mode with expected PPS interval of %d milliseconds\n",
			device->interval);

		/* use a status pin that's low besides ACK */
		read_status = port->ops->read_status;
		sta = read_status(port);

		local_irq_save(flags);
		sched_clock_idle_sleep_event();

		ktime_get_raw_ts64(&start);
		ktime_get_raw_ts64(&end);
		ktime_get_raw_ts64(&start);
		pollct = LOOP_CAL_COUNT;
		if ((sta & PARPORT_STATUS_BUSY) == 0) {
			if (static_branch_likely(&direct)) {
				while (pollct-- && (inb(s_addr) &
					PARPORT_STATUS_BUSY) == 0) {}
			} else {
				while (pollct-- && (read_status(port) &
					PARPORT_STATUS_BUSY) == 0) {}
			}
		} else if ((sta & PARPORT_STATUS_SELECT) == 0) {
			if (static_branch_likely(&direct)) {
				while (pollct-- && (inb(s_addr) &
					PARPORT_STATUS_SELECT) == 0) {}
			} else {
				while (pollct-- && (read_status(port) &
					PARPORT_STATUS_SELECT) == 0) {}
			}
		}
		ktime_get_raw_ts64(&end);
		local_irq_restore(flags);
		if (pollct == LOOP_CAL_COUNT) {
			pr_err("%s must have BUSY or SELECT_IN line low(0V) to check polling speed, but both were high(5V), aborting\n",
				port->name);
			goto err_release_source;
		}
		if (pollct > 0) {
			pr_err("%s %s pin went high during polling speed check after %d polls, aborting\n",
				port->name, ((sta & PARPORT_STATUS_BUSY) == 0)?
				"BUSY":"SELECT_IN", pollct);
			goto err_release_source;
		}
		start_ns = timespec64_to_ns(&start);
		end_ns = timespec64_to_ns(&end);
		len = timespec64_sub(end,start);
		len_ns = timespec64_to_ns(&len);
		if (len_ns < LOOP_CAL_COUNT) {
			pr_info("%s took less than 1ns per poll! (%lld ns for %d polls), clamping to 1ns per poll\n",
				port->name, len_ns, LOOP_CAL_COUNT);
			device->poll_trys = POLL_START_US * 2 * 1000;
		} else {
			ns_perpoll = len_ns / LOOP_CAL_COUNT;
			if (ns_perpoll > 20000) {
				pr_err("%s took more than 20us per poll(%lld ns for %d polls), port may be invalid, aborting\n",
					port->name, len_ns, LOOP_CAL_COUNT);
				goto err_release_source;
			}
			/*
			 * double the start offset to give some time
			 * for delayed hrt wakeup
			 */
			device->poll_trys = (POLL_START_US * 2 * 1000)
				/ (len_ns / LOOP_CAL_COUNT);
		}
		pr_info("%s %d polls completed in %lld ns, loop count: %u\n",
			port->name, LOOP_CAL_COUNT, len_ns, device->poll_trys);
		if (device->poll_trys < MIN_TRYS) {
			pr_err("%s loop count of %u must be more than %u, aborting\n",
				port->name, device->poll_trys, MIN_TRYS);
			goto err_release_source;
		}
		device->poll_target = device->poll_trys / 2;
		device->prev_pollct = device->poll_target;

		/* interrupt mask time for polling loop failure */
		device->mask_jiffies = msecs_to_jiffies(device->interval - 100);

		hrtimer_init(&device->hrt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		device->hrt.function = pps_hrt;
		device->hrt_period_ns = device->interval * 1000000;
		device->hrt_period = ns_to_ktime(device->hrt_period_ns);
		device->hrt_initial_period = ns_to_ktime( 
			device->hrt_period_ns - (POLL_START_US * 1000));
	} else { /* PM_INT */
		device->mask_jiffies = msecs_to_jiffies(ppps_mask[port_num]);
	}

	if (ppps_echo) {
		pollct = LOOP_CAL_COUNT;

		local_irq_save(flags);
		sched_clock_idle_sleep_event();

		if (static_branch_likely(&direct)) {
			ktime_get_raw_ts64(&end);
			ktime_get_raw_ts64(&start);
			while (pollct--)
				outb(ECHO_DATA_LO, d_addr);
			ktime_get_raw_ts64(&end);
		} else {
			write_data = port->ops->write_data;
			ktime_get_raw_ts64(&end);
			ktime_get_raw_ts64(&start);
			while (pollct--)
				write_data(port, ECHO_DATA_LO);
			ktime_get_raw_ts64(&end);
		}
		local_irq_restore(flags);
		start_ns = timespec64_to_ns(&start);
		end_ns = timespec64_to_ns(&end);
		len = timespec64_sub(end,start);
		len_ns = timespec64_to_ns(&len);
		ns_perpoll = len_ns / LOOP_CAL_COUNT;
		pr_info("%s %d port output writes completed in %lld ns\n",
			port->name, LOOP_CAL_COUNT, len_ns);
	}

	device->wq = create_singlethread_workqueue("pps_wq");
	if (device->wq == NULL) {
		pr_err("%s couldn't create delay queue\n", port->name);
		goto err_release_source;
	}
	INIT_DELAYED_WORK(&device->dw, pps_work);

	device->index = index;

	/* enable interrupt */
	queue_delayed_work(device->wq, &device->dw, STARTUP_DELAY);

	return;

err_release_source:
	port->ops->disable_irq(port);
	pps_unregister_source(device->pps);
	device->pps = NULL;
err_release_dev:
	parport_release(device->pardev);
err_unregister_dev:
	parport_unregister_device(device->pardev);
	/*
	 * can't kfree here because parport_detach will be called later during
	 * module unload
	 */
	return;
err_free:
	ida_simple_remove(&pps_client_index, index);
	kfree(device);
}

static void parport_detach(struct parport *port)
{
	struct pardevice *pardev = port->cad;
	struct pps_client_pp *device;

	/* FIXME: oooh, this is ugly! */
	if (!pardev || strcmp(pardev->name, KBUILD_MODNAME))
		/* not our port */
		return;

	port->ops->disable_irq(port);

	device = pardev->private;
	if (device->wq) {
		if (device->mode != PM_INT) {
			device->mode = PM_SHUTDOWN; /* flag hrt callback */
			/* wait in case polling is in progress */
			msleep(1100);
			hrtimer_cancel(&device->hrt);
		}
		cancel_delayed_work_sync(&device->dw); 
		flush_workqueue(device->wq);
		destroy_workqueue(device->wq);
	}
	/* wait in case interrupt handler is running on another CPU */
	msleep(100);

	if (device->pps)
		pps_unregister_source(device->pps);

	parport_release(pardev);
	parport_unregister_device(pardev);
	ida_simple_remove(&pps_client_index, device->index);
	kfree(device);
}

static struct parport_driver pps_parport_driver = {
	.name = KBUILD_MODNAME,
	.match_port = parport_attach,
	.detach = parport_detach,
	.devmodel = true,
};

/* module stuff */

static int __init pps_parport_init(void)
{
	int ret;

	pr_info(DRVDESC "\n");

	if (ppps_direct) {
		pr_info("using port IO instructions directly\n");
		static_branch_enable(&direct);
	}
	if (ppps_echo) {
		pr_info("echo on D0-D7 lines enabled\n");
		static_branch_enable(&echo);
	}

	ret = parport_register_driver(&pps_parport_driver);
	if (ret) {
		pr_err("unable to register with parport\n");
		return ret;
	}

	return  0;
}

static void __exit pps_parport_exit(void)
{
	parport_unregister_driver(&pps_parport_driver);
}

module_init(pps_parport_init);
module_exit(pps_parport_exit);

MODULE_AUTHOR("Alexander Gordeev <lasaine@lvk.cs.msu.su>");
MODULE_DESCRIPTION(DRVDESC);
MODULE_LICENSE("GPL");
