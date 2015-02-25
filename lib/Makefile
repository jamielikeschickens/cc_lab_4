CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c STM32L1xx_StdPeriph_Driver/src
vpath %.c STMTouch_Driver/src

CFLAGS  = -g -O2 -Wall
CFLAGS += -mcpu=cortex-m3 -mthumb --specs=nano.specs -nostartfiles 
CFLAGS += -ICMSIS/Include -ICMSIS/Device/ST/STM32L1xx/Include -ISTM32L1xx_StdPeriph_Driver/inc
CFLAGS += -ISTMTouch_Driver/inc

SRCS = stm32l1xx_adc.c stm32l1xx_comp.c stm32l1xx_crc.c \
	   	stm32l1xx_dac.c stm32l1xx_dbgmcu.c stm32l1xx_dma.c stm32l1xx_exti.c \
			stm32l1xx_flash.c stm32l1xx_gpio.c stm32l1xx_i2c.c stm32l1xx_iwdg.c \
				misc.c stm32l1xx_lcd.c stm32l1xx_pwr.c stm32l1xx_rcc.c stm32l1xx_rtc.c \
					stm32l1xx_spi.c stm32l1xx_syscfg.c stm32l1xx_tim.c \
						stm32l1xx_usart.c stm32l1xx_wwdg.c

SRCS += tsl.c tsl_acq.c \
		tsl_dxs.c tsl_ecs.c tsl_filter.c tsl_globals.c tsl_linrot.c \
		tsl_object.c tsl_time.c tsl_touchkey.c tsl_acq_stm32l1xx_sw.c \
		tsl_time_stm32l1xx.c

OBJS = $(SRCS:.c=.o)

.PHONY: libstm32l1.a

all: libstm32l1.a

%.o : %.c
		$(CC) $(CFLAGS) -c -o $@ $^

libstm32l1.a: $(OBJS)
		$(AR) -r $@ $(OBJS)

clean:
		rm -f $(OBJS) libstm32l1.a

