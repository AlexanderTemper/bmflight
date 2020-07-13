APP_NAME := serialTest
CSRCS := serialTest.c

UNIT_TEST_DIR := ../..
SRC_DIR := ../../../../src



# Include PATHS
INC_PATH := \
	$(SRC_DIR)/

include $(UNIT_TEST_DIR)/test_common.mk


CSRCS += \
	$(SRC_DIR)/drivers/serial.c \
	stopWatch.c
	
# Create object files list from source files list.
obj-y := $(addprefix $(BUILD_DIR)/$(APP_NAME)/, $(addsuffix .o,$(basename $(CSRCS))))

all: $(EXE_FILE)

# Link
$(EXE_FILE): $(obj-y)
	@$(LD) -o $@ $(obj-y) $(LDFLAGS) $(LDLIBS)
	
$(BUILD_DIR)/$(APP_NAME)/%.o: %.c
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS)  -c $< -o $@