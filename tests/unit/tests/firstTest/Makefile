APP_NAME := testUnitTest
CSRCS := testUnitTest.c

UNIT_TEST_DIR := ../..
include $(UNIT_TEST_DIR)/test_common.mk

# Create object files list from source files list.
obj-y := $(addprefix $(BUILD_DIR)/$(APP_NAME)/, $(addsuffix .o,$(basename $(CSRCS))))

all: $(EXE_FILE)

# Link
$(EXE_FILE): $(obj-y)
	@$(LD) -o $@ $(obj-y) $(LDFLAGS) $(LDLIBS)
	
$(BUILD_DIR)/$(APP_NAME)/%.o: %.c
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS)  -c $< -o $@