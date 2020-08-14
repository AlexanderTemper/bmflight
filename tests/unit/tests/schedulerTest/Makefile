APP_NAME := schedulerTest
CSRCS := schedulerTest.c

UNIT_TEST_DIR := ../..
SRC_DIR := ../../../..

# Include PATHS
INC_PATH := \
	$(SRC_DIR)/src/

include $(UNIT_TEST_DIR)/test_common.mk
CFLAGS += -DEBUG -O0 -g3 -gdwarf-2
# Files to build for source
CSRCS_SRC := \
	src/common/debug.c \
	src/common/time.c \
	src/scheduler/scheduler.c \
	
	
obj-y := $(addprefix $(BUILD_DIR)/$(APP_NAME)/, $(addsuffix .o,$(basename $(CSRCS_SRC))))	

#Files For Test
CSRCS += \
#	stopWatch.c
	
# Create object files list from source files list.
obj-y += $(addprefix $(BUILD_DIR)/$(APP_NAME)/, $(addsuffix .o,$(basename $(CSRCS))))

all: $(EXE_FILE)

# Link
$(EXE_FILE): $(obj-y)
	@$(LD) -o $@ $(obj-y) $(LDFLAGS) $(LDLIBS)
	

$(BUILD_DIR)/$(APP_NAME)/%.o: %.c
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS)  -c $< -o $@
	
	
$(BUILD_DIR)/$(APP_NAME)/%.o: $(SRC_DIR)/%.c
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS)  -c $< -o $@