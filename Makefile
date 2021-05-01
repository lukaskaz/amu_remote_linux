#===================================================================#
#Output files
APP_NAME=project
EXECUTABLE=obj/$(APP_NAME).elf
MAP_FILE=out/$(APP_NAME).map
MAP_FILE_LD=$(MAP_FILE).ld
HEX_IMAGE=out/$(APP_NAME).hex
USROBJ_DIR=obj/
USRINC_DIR=user/inc/
USRSRC_DIR=user/src/

#======================================================================#
#Cross Compiler
CROSSCOMPILER_PATH=../tools/gcc-arm-none-eabi-10-2020-q4-major/bin/
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
AR=arm-none-eabi-ar
READELF=arm-none-eabi-readelf
STRIP=arm-none-eabi-strip

#======================================================================#
#Flags
CFLAGS=-Wall -Wno-unused-function -std=gnu17
CFLAGS+=-mlittle-endian -mthumb -mcpu=cortex-m3 -mfloat-abi=soft
CFLAGS+=-ffreestanding -ffunction-sections -fdata-sections --specs=nano.specs
CFLAGS+=-D STM32F10X_MD
CFLAGS+=-D USE_STDPERIPH_DRIVER
CFLAGS+=-D MALLOC_PROVIDED
CFLAGS+=-u _printf_float

#stm32-flash
LINKER_SCR=scripts/stm32_flash.ld
LDFLAGS=$(CFLAGS) 
LDFLAGS+=-Wl,--gc-sections -Wl,-Map=$(MAP_FILE_LD)
LDFLAGS+=-T $(LINKER_SCR)
ADDLIBS=-lm

#archiver options
ARFLAGS=-cr

#ELF interpreter options
RELFFLAGS=-aW

#Bin file transformer options
OCPYFLAGS=-O ihex

#======================================================================#
#Libraries
#Stm32 libraries
ST_LIB=lib/StdPeriph/STM32F10x

#FreeRTOS OS libraries
OS_LIB=os/FreeRTOS

#USB libraries
USB_LIB=lib/USB/STM32F10x

#CMSIS libraries
CFLAGS+=-I lib/CMSIS/CM3/CoreSupport/

#StdPeriph includes
CFLAGS+=-I $(ST_LIB)/inc/
CFLAGS+=-I lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/

#USB includes
CFLAGS+=-I $(USB_LIB)/usb_vcp/inc
CFLAGS+=-I $(USB_LIB)/usb_device/inc

#FreeRTOS includes
CFLAGS+=-I $(OS_LIB)
CFLAGS+=-I $(OS_LIB)/include
CFLAGS+=-I $(OS_LIB)/portable/GCC/ARM_CM3

#Misc includes
CFLAGS+=-I system
CFLAGS+=-I user/inc

#======================================================================#
#setup system clock
SRC=system/system_stm32f10x.c

#setup interrupts handlers
SRC+=system/stm32f10x_it.c
SRC+=lib/CMSIS/CM3/CoreSupport/core_cm3.c

#add main code
SRC+=os/main.c
SRC+=system/syscalls.c
SRC+=system/sysmem_rtos.c
#SRC+=system/printf-stdarg.c
USRSRC+=mod_led.c
USRSRC+=mod_sound.c
USRSRC+=mod_sensors.c
USRSRC+=mod_drive.c
USRSRC+=mod_orientsensor.c
USRSRC+=mod_lcd.c
USRSRC+=mod_lighting.c
USRSRC+=mod_console.c
USRSRC+=mod_i2c.c
USRSRC+=mod_radio.c
USRSRC+=mod_auxiliary.c

USROBJ=$(USRSRC:.c=.o)
USROBJ_GROUP=$(addprefix $(USROBJ_DIR),$(USROBJ))

#StdPeriph
LIBSTM32_OUT = lib/libstm32.a
LIBSTM32_OBJS = $(ST_LIB)/src/misc.o \
            $(ST_LIB)/src/stm32f10x_rcc.o \
            $(ST_LIB)/src/stm32f10x_gpio.o \
            $(ST_LIB)/src/stm32f10x_exti.o \
            $(ST_LIB)/src/stm32f10x_tim.o \
            $(ST_LIB)/src/stm32f10x_adc.o \
            $(ST_LIB)/src/stm32f10x_usart.o \
            $(ST_LIB)/src/stm32f10x_pwr.o \
            $(ST_LIB)/src/stm32f10x_dma.o

#FreeRTOS
LIBFREERTOS_OUT = lib/libfreertos.a
LIBFREERTOS_OBJS = $(OS_LIB)/tasks.o \
                   $(OS_LIB)/list.o \
                   $(OS_LIB)/queue.o \
                   $(OS_LIB)/timers.o \
                   $(OS_LIB)/croutine.o \
                   $(OS_LIB)/portable/GCC/ARM_CM3/port.o \
                   $(OS_LIB)/portable/MemMang/heap_4.o

#USB
LIBUSB_OUT = lib/libusb.a
LIBUSB_OBJS = $(USB_LIB)/usb_vcp/src/hw_config.o \
              $(USB_LIB)/usb_vcp/src/usb_desc.o \
              $(USB_LIB)/usb_vcp/src/usb_endp.o \
              $(USB_LIB)/usb_vcp/src/usb_istr.o \
              $(USB_LIB)/usb_vcp/src/usb_prop.o \
              $(USB_LIB)/usb_vcp/src/usb_pwr.o \
              $(USB_LIB)/usb_device/src/usb_core.o \
              $(USB_LIB)/usb_device/src/usb_init.o \
              $(USB_LIB)/usb_device/src/usb_int.o \
              $(USB_LIB)/usb_device/src/usb_mem.o \
              $(USB_LIB)/usb_device/src/usb_regs.o \
              $(USB_LIB)/usb_device/src/usb_sil.o

#======================================================================#
#STM32 startup file
#STARTUP=lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s
STARTUP=system/startup_stm32f10x_md.s

#======================================================================#
#Make rules

SHELL:=/bin/bash
PATH:=$(CROSSCOMPILER_PATH):$(PATH)

debug: CFLAGS+=-O2 -g3
build: CFLAGS+=-O2
release: CFLAGS+=-O3

build: $(MAP_FILE) $(HEX_IMAGE) 
rebuild: clean build
release: clean strip $(MAP_FILE) $(HEX_IMAGE)
debug: clean $(MAP_FILE) $(HEX_IMAGE)


$(HEX_IMAGE):$(EXECUTABLE) 
	$(OBJCOPY) $(OCPYFLAGS) $^ $@

$(LIBSTM32_OUT): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)

$(LIBFREERTOS_OUT): $(LIBFREERTOS_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBFREERTOS_OBJS)

$(LIBUSB_OUT): $(LIBUSB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBUSB_OBJS)

$(EXECUTABLE): $(USROBJ_GROUP) $(SRC) $(STARTUP) $(LIBUSB_OUT) $(LIBFREERTOS_OUT) $(LIBSTM32_OUT)
	$(LD) $(LDFLAGS) $^ -o $@ $(ADDLIBS)

$(USROBJ_DIR)%.o: $(USRSRC_DIR)%.c $(USRINC_DIR)%.h
	@echo "Creating object '$(@F)'"
	$(CC) -c $< -o $@ $(CFLAGS)

$(MAP_FILE): $(EXECUTABLE)
	$(READELF) $(RELFFLAGS) $^ > $@

strip: $(EXECUTABLE)
	$(STRIP) -s $^

sizes_all:
	@grep -A1 "Symbol table" $(MAP_FILE)
	@egrep "[0-9]{1,8}:.* [0-9]{1,8} OBJECT" $(MAP_FILE) | sort -n -k 3 -r

sizes:
	@$(MAKE) --no-print-directory sizes_all | head -12	

sections:
	@egrep "\[.*\]" $(MAP_FILE)	

flash: 
	tools/flash_remote.sh $(HEX_IMAGE)

clean:
	rm -f $(USROBJ_DIR)/*
	rm -f $(EXECUTABLE) $(HEX_IMAGE)
	rm -f $(LIBFREERTOS_OUT) $(LIBSTM32_OUT) $(ST_LIB)/src/*.o
	rm -f $(OS_LIB)/portable/MemMang/*.o $(OS_LIB)/*.o $(OS_LIB)/portable/GCC/ARM_CM3/*.o
	rm -f $(LIBUSB_OUT) $(LIBUSB_OBJS) $(USB_LIB)/usb_vcp/src/*.o $(USB_LIB)/usb_device/src/*.o

#======================================================================
.PHONY: build rebuild clean flash strip sizes sizes_all sections flash

# END OF FILE
#%%%%%%%%%%%%
