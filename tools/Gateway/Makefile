APP_NAME := gateway
FC_SRC_DIR := ../../

BUILD_DIR := build
EXE_FILE := bin/$(APP_NAME).out

MSG_MKDIR = "MKDIR   $(dir $@)"
#------------------------------ Compiler --------------------------------
CC := gcc
#------------------------------ Linker ----------------------------------
LD := gcc

# Dependency file flags.
DEPFLAGS = -MD -MP -MQ $@

# Use pipes instead of temporary files for communication between processes
CFLAGS  += -pipe
LDFLAGS += -pipe

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
CFLAGS += -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS += -Wshadow -Wbad-function-cast -Wwrite-strings
CFLAGS += -Wsign-compare -Waggregate-return
CFLAGS += -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wpacked -Wredundant-decls -Wnested-externs -Wlong-long
CFLAGS += -Wunreachable-code
CFLAGS += -Wcast-align
CFLAGS += --param max-inline-insns-single=500

CFLAGS += -DEBUG -O0 -g3 -gdwarf-2
# Include PATHS
INC_PATH := \
	$(FC_SRC_DIR)src \

CFLAGS = \
   -D FC_TARGET_SITL
	
# Files to build for source
CSRCS := \
	main.c      \
	uart_serial \
	joy.c
	
	
CSRCS_FC_SRC := \
	src/common/time.c \
	src/common/streambuf.c \
	src/scheduler/scheduler.c \
	src/io/serial.c \
	src/msp/msp.c \
	src/common/debug.c \
	src/drivers/SITL/serial_tcp.c \
	
	# Include PATHS
INC_PATH += $(TEST_FRAMEWORK) 

# Add inlcude paths to preprocessor
CPPFLAGS  += $(foreach INC,$(addprefix ./,$(INC_PATH)),-I$(INC))


obj-y := $(addprefix $(BUILD_DIR)/, $(addsuffix .o,$(basename $(CSRCS_FC_SRC))))	

# Create object files list from source files list.
obj-y += $(addprefix $(BUILD_DIR)/, $(addsuffix .o,$(basename $(CSRCS))))

all: $(EXE_FILE)

# Link
$(EXE_FILE): $(obj-y)
	@$(LD) -o $@ $(obj-y) $(LDFLAGS) $(LDLIBS)
	

$(BUILD_DIR)/%.o: %.c
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS)  -c $< -o $@
	
	
$(BUILD_DIR)/%.o: $(FC_SRC_DIR)/%.c
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS)  -c $< -o $@
	
	
clean:
	rm -rf $(BUILD_DIR)