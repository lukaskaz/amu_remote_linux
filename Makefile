#===================================================================#
#Output files
APP_NAME=project
EXECUTABLE=obj/$(APP_NAME).elf
MAP_FILE=out/$(APP_NAME).map
HEX_IMAGE=out/$(APP_NAME).hex

#======================================================================#
#Cross Compiler
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
AR=arm-none-eabi-ar
READELF=arm-none-eabi-readelf

#======================================================================#
#Flags

#CFLAGS=-Wall -g -mlittle-endian -mthumb -ffreestanding --specs=nano.specs
CFLAGS=-Wall -O2 -mlittle-endian -mthumb -ffreestanding --specs=nano.specs -DMALLOC_PROVIDED
#CFLAGS=-Wall -g -O0 -mlittle-endian -mthumb --specs=nano.specs 
#CFLAGS=-Wall -g -O0 -mlittle-endian -mthumb -ffreestanding
CFLAGS+=-mcpu=cortex-m3 -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections 
#CFLAGS+=-mcpu=cortex-m3 -mfloat-abi=soft 
CFLAGS+=-D USE_STDPERIPH_DRIVER
CFLAGS+=-D STM32F10X_MD
CFLAGS+=-I./src/system/

#stm32-flash
LINKER_SCR=scripts/stm32_flash.ld
CFLAGS+=-Wl,-T,$(LINKER_SCR)

#archiver options
ARFLAGS=cr
#======================================================================#
#Libraries

#Stm32 libraries
ST_LIB=lib/STM32F10x_StdPeriph_Driver

#CMSIS libraries
CFLAGS+=-I lib/CMSIS/CM3/CoreSupport/

#StdPeriph includes
CFLAGS+=-I $(ST_LIB)/inc/
CFLAGS+=-I lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/


CFLAGS+=-I src/FreeRTOS
CFLAGS+=-I src/FreeRTOS/include
CFLAGS+=-I src/FreeRTOS/portable/GCC/ARM_CM3
CFLAGS+=-I src/modules/inc 
CFLAGS+=-I lib/USB/inc
CFLAGS+=-I lib/STM32_USB-FS-Device_Driver/inc
#======================================================================#
#setup system clock
SRC=src/system/system_stm32f10x.c

#setup interrupts handlers
SRC+=src/system/stm32f10x_it.c
#SRC+=lib/CMSIS/CM3/CoreSupport/core_cm3.c

#add main code
SRC+=src/main.c
#SRC+=src/newlib_stubs.c 
SRC+=src/alloc_stubs.c
SRC+=src/printf-stdarg.c
SRC+=src/modules/src/mod_signaling_led.c
SRC+=src/modules/src/mod_sound_signal.c
SRC+=src/modules/src/mod_sensors.c
SRC+=src/modules/src/mod_drive.c
SRC+=src/modules/src/mod_orientation_sensor.c
SRC+=src/modules/src/mod_lcd.c
SRC+=src/modules/src/mod_lighting.c
SRC+=src/modules/src/mod_console.c
SRC+=src/modules/src/mod_i2c.c
SRC+=src/modules/src/mod_radio_control.c

#StdPeriph

LIBSTM32_OUT = lib/libstm32.a
LIBSTM32_OBJS = $(ST_LIB)/src/misc.o \
	        $(ST_LIB)/src/stm32f10x_rcc.o \
	        $(ST_LIB)/src/stm32f10x_gpio.o \
        	$(ST_LIB)/src/stm32f10x_exti.o \
	        $(ST_LIB)/src/stm32f10x_tim.o \
        	$(ST_LIB)/src/stm32f10x_flash.o \
        	$(ST_LIB)/src/stm32f10x_adc.o \
        	$(ST_LIB)/src/stm32f10x_usart.o \
        	$(ST_LIB)/src/stm32f10x_pwr.o \
        	$(ST_LIB)/src/stm32f10x_dma.o \
		src/FreeRTOS/tasks.o \
	         src/FreeRTOS/list.o \
	         src/FreeRTOS/queue.o \
	         src/FreeRTOS/timers.o \
	         src/FreeRTOS/croutine.o \
	         src/FreeRTOS/portable/GCC/ARM_CM3/port.o \
	         src/FreeRTOS/portable/MemMang/heap_4.o \
	      lib/USB/src/hw_config.o \
	      lib/USB/src/usb_desc.o \
	      lib/USB/src/usb_endp.o \
	      lib/USB/src/usb_istr.o \
	      lib/USB/src/usb_prop.o \
	      lib/USB/src/usb_pwr.o \
	      lib/STM32_USB-FS-Device_Driver/src/usb_core.o \
	      lib/STM32_USB-FS-Device_Driver/src/usb_init.o \
	      lib/STM32_USB-FS-Device_Driver/src/usb_int.o \
	      lib/STM32_USB-FS-Device_Driver/src/usb_mem.o \
	      lib/STM32_USB-FS-Device_Driver/src/usb_regs.o \
	      lib/STM32_USB-FS-Device_Driver/src/usb_sil.o \



#======================================================================#
#STM32 startup file
STARTUP=lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s

#======================================================================#
#Make rules

#Make build
build:$(MAP_FILE) $(HEX_IMAGE) 

$(HEX_IMAGE):$(EXECUTABLE) 
	$(OBJCOPY) -O ihex $^ $@

$(LIBSTM32_OUT): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)

$(EXECUTABLE):$(SRC) $(STARTUP) $(LIBSTM32_OUT) 
	$(CC) $(CFLAGS) $^ -o $@

$(MAP_FILE): $(EXECUTABLE)
	$(READELF) -a $^ > $@

test: $(LIBFREEOS_OUT)

$(LIBFREEOS_OUT): $(LIBFREEOS_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBFREEOS_OBJS)

test1: $(LIBUSB_OUT)

$(LIBUSB_OUT): $(LIBUSB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBUSB_OBJS)


rebuild: clean build

#Make clean
clean:
	rm -rf $(EXECUTABLE) $(HEX_IMAGE) $(LIBSTM32_OUT) $(ST_LIB)/src/*.o src/FreeRTOS/portable/MemMang/*.o src/FreeRTOS/*.o src/FreeRTOS/portable/GCC/ARM_CM3/*.o
	rm -rf $(LIBUSB_OUT) lib/USB/src/*.o lib/STM32_USB-FS-Device_Driver/src/*.o

#Make flash
flash:
	tools/flash_remote.sh $(HEX_IMAGE)

openocd:
	openocd -f "../commom/openocd.cfg"
gdb:
	arm-none-eabi-gdb -x ../commom/gdb_init.gdb
gdbtui:
	arm-none-eabi-gdb -tui -x ../commom/gdb_init.gdb
#======================================================================
.PHONY:all clean flash
