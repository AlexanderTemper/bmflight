
SAMD_LIB_PATH = lib/SAMD20J18

#-------------------------------------ASF FILES---------------------------
ASF_PATH = $(SAMD_LIB_PATH)/ASF

ASF_INC_PATH = \
	$(ASF_PATH)/common                                             \
	$(ASF_PATH)/common/utils                                       \
	$(ASF_PATH)/common/utils/interrupt                             \
	$(ASF_PATH)/common/boards                                      \
	$(ASF_PATH)/common2/boards/user_board                          \
	$(ASF_PATH)/sam0/drivers/nvm                                   \
	$(ASF_PATH)/sam0/drivers/system                                \
	$(ASF_PATH)/sam0/drivers/system/clock                          \
	$(ASF_PATH)/sam0/drivers/system/clock/clock_samd20             \
	$(ASF_PATH)/sam0/drivers/system/interrupt                      \
	$(ASF_PATH)/sam0/drivers/system/interrupt/system_interrupt_samd20 \
	$(ASF_PATH)/sam0/drivers/system/pinmux                         \
	$(ASF_PATH)/sam0/drivers/system/power                          \
	$(ASF_PATH)/sam0/drivers/system/power/power_sam_d_r_h          \
	$(ASF_PATH)/sam0/drivers/system/reset                          \
	$(ASF_PATH)/sam0/drivers/system/reset/reset_sam_d_r_h          \
	$(ASF_PATH)/sam0/drivers/port                                  \
	$(ASF_PATH)/sam0/drivers/sercom                                \
	$(ASF_PATH)/sam0/drivers/sercom/spi                            \
	$(ASF_PATH)/sam0/drivers/sercom/usart                          \
	$(ASF_PATH)/sam0/drivers/sercom/i2c                            \
	$(ASF_PATH)/sam0/drivers/sercom/i2c/i2c_samd20                 \
	$(ASF_PATH)/sam0/drivers/tc                                    \
	$(ASF_PATH)/sam0/services/eeprom/emulator/main_array           \
	$(ASF_PATH)/sam0/utils                                         \
	$(ASF_PATH)/sam0/utils/syscalls                                \
	$(ASF_PATH)/sam0/utils/syscalls/gcc                            \
	$(ASF_PATH)/sam0/utils/make                                    \
	$(ASF_PATH)/sam0/utils/cmsis/samd20                            \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include                    \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source                     \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include/component          \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include/instance           \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/include/pio                \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source/gcc                 \
	$(ASF_PATH)/sam0/utils/header_files                            \
	$(ASF_PATH)/sam0/utils/preprocessor                            \
	$(ASF_PATH)/thirdparty/CMSIS/Include

	
ASF_CSRCS = \
	$(ASF_PATH)/common/utils/interrupt/interrupt_sam_nvic.c        \
	$(ASF_PATH)/common2/boards/user_board/init.c                   \
	$(ASF_PATH)/sam0/drivers/port/port.c                           \
	$(ASF_PATH)/sam0/drivers/sercom/sercom.c                       \
	$(ASF_PATH)/sam0/drivers/sercom/sercom_interrupt.c             \
	$(ASF_PATH)/sam0/drivers/sercom/spi/spi.c                      \
	$(ASF_PATH)/sam0/drivers/sercom/spi/spi_interrupt.c            \
	$(ASF_PATH)/sam0/drivers/sercom/usart/usart_interrupt.c        \
	$(ASF_PATH)/sam0/drivers/sercom/usart/usart.c                  \
	$(ASF_PATH)/sam0/drivers/sercom/i2c/i2c_samd20/i2c_master.c    \
	$(ASF_PATH)/sam0/drivers/system/clock/clock_samd20/clock.c     \
	$(ASF_PATH)/sam0/drivers/system/clock/clock_samd20/gclk.c      \
	$(ASF_PATH)/sam0/drivers/system/interrupt/system_interrupt.c   \
	$(ASF_PATH)/sam0/drivers/system/pinmux/pinmux.c                \
	$(ASF_PATH)/sam0/drivers/system/system.c                       \
	$(ASF_PATH)/sam0/drivers/tc/tc_sam_d_r_h/tc.c                  \
	$(ASF_PATH)/sam0/drivers/tc/tc_interrupt.c                     \
	$(ASF_PATH)/sam0/services/eeprom/emulator/main_array/eeprom.c  \
	$(ASF_PATH)/sam0/utils/syscalls/gcc/syscalls.c                 \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source/gcc/startup_samd20.c \
	$(ASF_PATH)/sam0/utils/cmsis/samd20/source/system_samd20.c 	
#-------------------------------------------------------------------------

EXE_FILE = bin/$(APP_NAME).elf
MAP_FILE = bin/$(APP_NAME).map


# Include PATHS
INC_PATH += \
	$(SAMD_LIB_PATH)               \
	$(SAMD_LIB_PATH)/ASF_Support   \
	$(SAMD_LIB_PATH)/drivers       \
	$(ASF_INC_PATH)                \
	src/drivers/SAMD20J18 		     
	

# List of C source files.
CSRCS += \
	$(ASF_CSRCS)                                 \
	$(SAMD_LIB_PATH)/ASF_Support/clock_support.c \
	$(SAMD_LIB_PATH)/ASF_Support/spi_support.c   \
	$(SAMD_LIB_PATH)/ASF_Support/i2c_support.c   \
	$(SAMD_LIB_PATH)/ASF_Support/tc_support.c    \
	$(SAMD_LIB_PATH)/drivers/bma2x2_support.c    \
	$(SAMD_LIB_PATH)/drivers/bma2x2.c            \
	$(SAMD_LIB_PATH)/drivers/bmg160_support.c    \
	$(SAMD_LIB_PATH)/drivers/bmg160.c            \
	$(SAMD_LIB_PATH)/drivers/bmm050_support.c    \
	$(SAMD_LIB_PATH)/drivers/bmm050.c
	
	
CSRCS += \
	src/drivers/SAMD20J18/serial.c  \
	src/drivers/SAMD20J18/platform.c

#------------------------------ Compiler --------------------------------
CC := arm-none-eabi-gcc
#------------------------------ Linker ----------------------------------
LD := arm-none-eabi-gcc

SIZE := arm-none-eabi-size

CFLAGS = \
       -D ARM_MATH_CM0PLUS=true \
       -D BOARD=USER_BOARD \
       -D SPI_CALLBACK_MODE=true \
       -D USART_CALLBACK_MODE=true \
       -D I2C_MASTER_CALLBACK_MODE=false \
       -D TC_ASYNC=true \
       
# Additional search paths for libraries.
LIB_PATH = $(ASF_PATH)/thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  arm_cortexM0l_math                                

# Extra flags to use when linking
#LDFLAGS = -u _printf_float
#LDFLAGS = -lm -lc -lnosys //TODO brauch ich die?
# CPU specific flags.
cpu_flags = -mcpu=cortex-m0plus -mthumb -D=__SAMD20J18__

# Add CPU Flags
CFLAGS  += $(cpu_flags)
LDFLAGS += $(cpu_flags)

# Add library
LDLIBS += -Wl,$(foreach LIB,$(LIBS),-l$(LIB))



# Add library search paths
LDFLAGS += -Wl,$(foreach _LIB_PATH,$(LIB_PATH),-L./$(_LIB_PATH))

# Output a link map file and a cross reference table
LDFLAGS += -Wl,-gc-sections,-Map,$(MAP_FILE)
LDFLAGS += -Wl,--cref
LDFLAGS += -Wl,--entry=Reset_Handler

# Linker file
LDFLAGS += -T./lib/SAMD20J18/link/samd20j18_flash.ld







