#------------------------------ Compiler --------------------------------
CC := gcc
#------------------------------ Linker ----------------------------------
LD := gcc

SIZE := size


#-------------------------------------------------------------------------

EXE_FILE = bin/SITL.elf
MAP_FILE = bin/SITL.map


# Include PATHS
INC_PATH += \
	src/drivers/SITL \
	lib/dyad 	

	
CSRCS += \
	src/drivers/SITL/serial_tcp.c  \
	src/drivers/SITL/platform.c    \
	lib/dyad/dyad.c