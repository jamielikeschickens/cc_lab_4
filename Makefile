SRCS = main.c stm32l_discovery_lcd.c  delay.s

PROJ_NAME = main

CC_PREFIX = arm-none-eabi-
CC = $(CC_PREFIX)gcc
OBJCOPY = $(CC_PREFIX)objcopy
GDB = $(CC_PREFIX)gdb
STARTUP = startup_stm32l1xx_md.s System_Init.s

STD_PERIPH_LIB = lib/STM32L1xx_StdPeriph_Driver/inc/

CFLAGS += -mcpu=cortex-m3 -mthumb -g --specs=nano.specs -nostartfiles -O0 -std=c99
CFLAGS += -Ilib/STM32L1xx_StdPeriph_Driver/inc -Ilib/CMSIS/Include -Ilib
CFLAGS += -Ilib/STMTouch_Driver/inc
CFLAGS += -Ilib/CMSIS/Device/ST/STM32L1xx/Include
CFALGS += -DUSE_STDPERIPH_DRIVER

LINKER_SCRIPT = ld/stm32.ld

## ST-UTIL ##
STUTIL = st-util
STFLASH = st-flash


## TARGETS ##
all: stdlib program

stdlib: $(STD_SRCS)
	$(MAKE) -C lib

program: $(SRCS)
	$(CC) $(CFLAGS) -T $(LINKER_SCRIPT) $^ $(STARTUP) -Llib -lstm32l1 -o $(PROJ_NAME).elf
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

debug: program $(SRCS)	
	$(GDB) $(PROJ_NAME).elf -ex="tar extended-remote :4242" -ex="load"	

flash: program 
	$(STFLASH) write $(PROJ_NAME).bin 0x08000000 

clean:
	rm $(PROJ_NAME).elf $(PROJ_NAME).bin

.PHONY: clean all
