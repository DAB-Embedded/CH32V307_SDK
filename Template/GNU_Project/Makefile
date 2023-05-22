# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# Author: DAB-Embedded
# ------------------------------------------------

######################################
# target
######################################
TARGET = Template


######################################
# building variables
######################################
# debug build?
ifdef DEBUG_BUILD
DEBUG = 1
endif

ifeq ($(DEBUG), 1)
# no optimization
OPT = -Og
else
# optimization
OPT = -Os
endif


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
../ch32v30x_it.c \
../main.c \
../system_ch32v30x.c \
../Peripheral/src/ch32v30x_adc.c \
../Peripheral/src/ch32v30x_bkp.c \
../Peripheral/src/ch32v30x_can.c \
../Peripheral/src/ch32v30x_crc.c \
../Peripheral/src/ch32v30x_dac.c \
../Peripheral/src/ch32v30x_dma.c \
../Peripheral/src/ch32v30x_dvp.c \
../Peripheral/src/ch32v30x_dbgmcu.c \
../Peripheral/src/ch32v30x_eth.c \
../Peripheral/src/ch32v30x_exti.c \
../Peripheral/src/ch32v30x_flash.c \
../Peripheral/src/ch32v30x_fsmc.c \
../Peripheral/src/ch32v30x_gpio.c \
../Peripheral/src/ch32v30x_i2c.c \
../Peripheral/src/ch32v30x_iwdg.c \
../Peripheral/src/ch32v30x_misc.c \
../Peripheral/src/ch32v30x_opa.c \
../Peripheral/src/ch32v30x_pwr.c \
../Peripheral/src/ch32v30x_rcc.c \
../Peripheral/src/ch32v30x_rng.c \
../Peripheral/src/ch32v30x_rtc.c \
../Peripheral/src/ch32v30x_sdio.c \
../Peripheral/src/ch32v30x_spi.c \
../Peripheral/src/ch32v30x_tim.c \
../Peripheral/src/ch32v30x_usart.c \
../Peripheral/src/ch32v30x_wwdg.c \
../Core/core_riscv.c \
../Utils/debug.c

# ASM sources
ASM_SOURCES =  \
../Startup/startup_ch32v30x_D8C.s


#######################################
# binaries
#######################################
PREFIX = riscv-none-embed-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################

# mcu
MCU = -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -msave-restore

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DGD32E230

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-I../Peripheral/inc \
-I../Core \
-I../Utils \
-I../

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = CH32V307VCT6_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -nostartfiles

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
