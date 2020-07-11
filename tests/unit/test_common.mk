TEST_FRAMEWORK := $(UNIT_TEST_DIR)/framework/sput-1.4.0
BUILD_DIR := $(UNIT_TEST_DIR)/build
EXE_FILE := $(UNIT_TEST_DIR)/bin/$(APP_NAME).out

MSG_MKDIR = "MKDIR   $(dir $@)"
#------------------------------ Compiler --------------------------------
CC := gcc
#------------------------------ Linker ----------------------------------
LD := gcc

# Include PATHS
INC_PATH += $(TEST_FRAMEWORK) 

# Add inlcude paths to preprocessor
CPPFLAGS  += $(foreach INC,$(addprefix ./,$(INC_PATH)),-I$(INC))

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

