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

	
CSRCS += \
	src/drivers/SITL/serial_tcp.c  \
	src/drivers/SITL/platform.c    \
	src/drivers/SITL/udplink.c    \
	
	
LDFLAGS += -lm -lpthread -lc -lrt