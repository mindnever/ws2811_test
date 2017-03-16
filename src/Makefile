

TOOLCHAIN=../../tools/gcc-arm-none-eabi/bin/arm-none-eabi-
STDPERIPH=../../libraries/STM32F10x_StdPeriph_Driver
CMSIS=../../libraries/CMSIS
BUILDDIR=../build

all: $(BUILDDIR)/firmware.elf

CC=$(TOOLCHAIN)gcc
LD=$(TOOLCHAIN)gcc


DEFINES = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DPIOS_INCLUDE_DELAY -DLED_STRIP -DSTM32F1
CFLAGS += -I$(STDPERIPH)/inc -I$(CMSIS)/Include  -I$(CMSIS)/Core/CM3 $(DEFINES) -I. -ggdb -mcpu=cortex-m3 -march=armv7-m -mfloat-abi=soft -mthumb -std=c99
LDFLAGS = -Wl,-T -Wl,link_stm32f10x_MD.ld -Wl,-Map -Wl,$(BUILDDIR)/firmware.map -nostartfiles

STDPERIPH_SRC = stm32f10x_rcc.c stm32f10x_gpio.c stm32f10x_dma.c stm32f10x_tim.c misc.c
CMSIS_SRC = system_stm32f10x.c startup/gcc/startup_stm32f10x_md.s

SRC = main.c pios_delay.c light_ws2811strip.c light_ws2811strip_stm32f10x.c colorconversion.c

$(BUILDDIR)/firmware.elf: $(SRC) $(addprefix $(STDPERIPH)/src/, $(STDPERIPH_SRC)) $(addprefix $(CMSIS)/Core/CM3/, $(CMSIS_SRC))
	$(CC) $(CFLAGS) $(LDFLAGS) $(abspath $^) -o $@


clean:
	rm -f $(BUILDDIR)/firmware.elf
