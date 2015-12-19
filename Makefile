#The MIT License (MIT)
#
#Copyright (c) 2015 Marco Russi
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.


#ATTENTON: modify following names and paths as required
PROJECT_NAME := ble_cube
NRFJPROG_PATH := /opt/nrfjprog
SDK_PATH := /opt/nRF51_SDK_10.0.0_dc26b5e
LINKER_SCRIPT := ble_cube_nrf51.ld
GNU_INSTALL_ROOT := /home/marco/ARMToolchain/gcc-arm-none-eabi-4_9-2015q2
GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi

export OUTPUT_FILENAME

SDK_COMPONENTS_PATH = $(SDK_PATH)/components
SDK_EXAMPLES_PATH = $(SDK_PATH)/examples
TEMPLATE_PATH = $(SDK_COMPONENTS_PATH)/toolchain/gcc
OUTPUT_FILENAME = $(PROJECT_NAME)_s130_pca10028

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'
GDB       	:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gdb"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/button/app_button.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/util/app_error.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/fifo/app_fifo.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/timer/app_timer.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/trace/app_trace.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/util/nrf_assert.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/uart/retarget.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/uart/app_uart_fifo.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/delay/nrf_delay.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/common/nrf_drv_common.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath util.c) \
$(abspath uart_manager.c) \
$(abspath conn_manager.c) \
$(abspath main.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/common/ble_advdata.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/common/ble_conn_params.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/ble_db_discovery/ble_db_discovery.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/ble_services/ble_nus_c/ble_nus_c.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/common/ble_srv_common.c) \
$(abspath $(SDK_COMPONENTS_PATH)/toolchain/system_nrf51.c) \
$(abspath $(SDK_COMPONENTS_PATH)/softdevice/common/softdevice_handler/softdevice_handler.c) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(SDK_COMPONENTS_PATH)/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS  = -I$(abspath config)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/util)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/toolchain/gcc)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/toolchain)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/ble/common)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/common)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/button)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/trace)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/config)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/fifo)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/uart)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/uart)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/device)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/ble/ble_db_discovery)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/softdevice/common/softdevice_handler)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/delay)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/timer)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/hal)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/ble/ble_services/ble_nus_c)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/softdevice/s130/headers)

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DBOARD_PCA10028
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DBSP_UART_SUPPORT
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DBSP_UART_SUPPORT
#default target - first one defined
default: clean nrf51422_xxac_s130

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac_s130

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac_s130
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxac_s130: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(NRFJPROG_PATH)/nrfjprog.sh --flash $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

erase: $(MAKECMDGOALS)
	@echo Erasing device...
	$(NRFJPROG_PATH)/nrfjprog.sh --erase-all

## Flash softdevice
flash_softdevice: 
	@echo Flashing: s110_softdevice.hex
	$(NRFJPROG_PATH)/nrfjprog.sh --flash-softdevice $(SDK_COMPONENTS_PATH)/softdevice/s130/hex/s130_nrf51_1.0.0_softdevice.hex
