
APP_NAME = BMF055FlightController


EXE_FILE = bin/$(APP_NAME).out

TARGET = BMF055
BUILD_DIR := build/

# Choose target MCU
ifeq ($(TARGET),$(filter $(TARGET),SITL))
TARGET_MCU := SITL
SIMULATOR_BUILD = yes

else ifeq ($(TARGET),BMF055)
TARGET_MCU := SAMD20J18

else
$(error Unknown target MCU specified.)
endif


# Include target makefile
include ./make/mcu/$(TARGET_MCU).mk

INC_PATH += \
	src
# List of C source files.
CSRCS += \
	src/main.c	           \
	src/common/debug.c     \
	src/common/streambuf.c \
	src/common/time.c      \
	src/io/serial.c        \
	src/msp/msp.c

# Add inlcude paths to preprocessor
CPPFLAGS  += $(foreach INC,$(addprefix ./,$(INC_PATH)),-I$(INC))

# Dependency file flags.
DEPFLAGS = -MD -MP -MQ $@

CFLAGS  += -pipe
LDFLAGS += -pipe -lm

# Always enable warnings. And be very careful about implicit
# declarations.
CFLAGS += -Wall -Wstrict-prototypes -Wmissing-prototypes
CFLAGS += -Werror-implicit-function-declaration

# IAR doesn't allow arithmetic on void pointers, so warn about that.
CFLAGS += -Wpointer-arith

# Compile C files using the GNU99 standard.
CFLAGS += -std=gnu99

# Don't use strict aliasing (very common in embedded applications).
CFLAGS += -fno-strict-aliasing

# Separate each function and data into its own separate section to allow
# garbage collection of unused sections.
CFLAGS += -ffunction-sections -fdata-sections

# Various cflags.

LDFLAGS += -Wl,--print-memory-usage

MSG_MKDIR = "MKDIR   $(dir $@)"


# Create object files list from source files list.
obj-y := $(addprefix $(BUILD_DIR), $(addsuffix .o,$(basename $(CSRCS))))


.PHONY: all debug release clean
all: debug

	
	
############### DEBUG BUILD ###########
debug:  CFLAGS += -DEBUG -O0 -g3 -gdwarf-2
debug::
	@echo "=========== Debug build for $(APP_NAME) ===="
debug:: $(EXE_FILE)


############## RELEASE BUILD ############
release:  CFLAGS += -DNDEBUG -Ofast
release::
	@echo "=========== Release Build for $(APP_NAME) ===="
release:: $(EXE_FILE)

# Link
$(EXE_FILE): $(obj-y)
	@echo "Linking" $@
	@$(LD) -o $@ $(obj-y) $(LDFLAGS) $(LDLIBS)
	$(SIZE) $(EXE_FILE)
	$(SIZE) -Ax $@
	$(SIZE) -Bx $@
	
$(BUILD_DIR)%.o: %.c
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS)  -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
	
