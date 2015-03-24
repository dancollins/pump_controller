#   Copyright 2015 Dan Collins & Tim Dawson
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

# Project Name
PROJECT = pump_controller

# Add project sources
SRCS_C = \
	main.c \
	system_stm32f0xx.c

SRCS_H =

SRCS_S = \
	startup_stm32f051.S

# Add project include directories here
INCLUDE = src

#
# NARM
#
# Include the NARM directory
ifndef NARM_DIR
$(error "NARM_DIR not defined")
endif

#
# VENDOR
#
# Include the STM code
ifndef STM_DIR
$(error "STM_DIR not defined")
endif

STM_INCLUDE = \
	STM32F0xx_StdPeriph_Lib_V1.3.1/Libraries/STM32F0xx_StdPeriph_Driver/inc \
	STM32F0xx_StdPeriph_Lib_V1.3.1/Libraries/CMSIS/Include \
	STM32F0xx_StdPeriph_Lib_V1.3.1/Libraries/CMSIS/Device/ST/STM32F0xx/Include

STM_DEFS = \
	-DUSE_STDPERIPH_DRIVER \
	-DSTM32F051

#
# COMPILER FLAGS
#
ARCH_FLAGS=-mthumb -mcpu=cortex-m0 -mfloat-abi=soft

GC=-Wl,--gc-sections
MAP=-Wl,-Map=$(PROJECT).map

CFLAGS += $(ARCH_FLAGS) $(STM_DEFS)
CFLAGS += -Wall -Werror -g -std=gnu99
CFLAGS += -Os -flto -ffunction-sections -fdata-sections
CFLAGS += -fno-builtin

CFLAGS += $(addprefix -I, $(INCLUDE))
CFLAGS += $(addprefix -I, $(NARM_DIR))
CFLAGS += $(addprefix -I$(STM_DIR)/, $(STM_INCLUDE))

LFLAGS += $(addprefix -L, $(NARM_DIR))
LFLAGS += $(addprefix -L$(STM_DIR)/, STM32F0xx_StdPeriph_Lib_V1.3.1/Libraries)
LFLAGS += -Tstm32f051k8_flash.ld
LFLAGS += --specs=nano.specs -lc -lnarm -lstm32f0xx -lnosys
LFLAGS += $(GC) $(MAP)

OBJS = \
	$(addprefix build/, $(SRCS_C:.c=.o)) \
	$(addprefix build/, $(SRCS_S:.S=.o))

LIBNARM = $(addprefix $(NARM_DIR)/, libnarm.a)

LIBSTM_DIR = $(addprefix $(STM_DIR)/, STM32F0xx_StdPeriph_Lib_V1.3.1/Libraries)
LIBSTM = $(addprefix $(LIBSTM_DIR)/, libstm32f0xx.a)

LIBS = $(LIBNARM) $(LIBSTM)


#
# BUILD RULES
#
.PHONY: all clean prog

all: output/$(PROJECT).elf

build/%.o: src/%.c
	$(MKDIR) -p $(dir $@)
	$(CC) $< $(CFLAGS) -c -o $@

build/%.o: src/%.S
	$(MKDIR) -p $(dir $@)
	$(CC) $< $(CFLAGS) -c -o $@

%libnarm.a:
	$(MAKE) -C $(NARM_DIR)

%libstm32f0xx.a:
	$(MAKE) -C $(LIBSTM_DIR)

output/$(PROJECT).elf: $(LIBS) $(OBJS)
	$(MKDIR) -p output
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@
	$(OBJCOPY) -O binary output/$(PROJECT).elf output/$(PROJECT).bin
	$(OBJCOPY) -O ihex output/$(PROJECT).elf output/$(PROJECT).hex
	$(OBJDUMP) -St -marm output/$(PROJECT).elf > output/$(PROJECT).lst
	$(SIZE) output/$(PROJECT).elf

clean:
	rm -rf build
	rm -rf output
	rm $(PROJECT).map

prog: all
	stm32flash -w output/$(PROJECT).bin -v -g 0x0 /dev/ttyUSB0

#
# TOOLCHAIN
#
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size
MKDIR=mkdir
MAKE=make

