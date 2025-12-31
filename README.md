Linux parallel port PPS client driver with an option to use polling instead of interrupts

replace pps_parport.c in drivers/pps/clients
optionally replace Makefile (adds optimization)
build as a module

current version is compatible with kernel version 6.18+
previous versions: 4.1 to 4.10, 4.11 to 6.16, 6.17

