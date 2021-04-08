Archive member included to satisfy reference by file (symbol)

lib/libusb.a(hw_config.o)     obj/mod_console.o (Set_System)
lib/libusb.a(usb_desc.o)      lib/libusb.a(hw_config.o) (Virtual_Com_Port_StringSerial)
lib/libusb.a(usb_endp.o)      obj/mod_console.o (USB_Rx_Counter)
lib/libusb.a(usb_istr.o)      /tmp/cchESTC9.o (USB_Istr)
lib/libusb.a(usb_prop.o)      lib/libusb.a(usb_istr.o) (Device_Property)
lib/libusb.a(usb_pwr.o)       lib/libusb.a(usb_prop.o) (PowerOn)
lib/libusb.a(usb_core.o)      lib/libusb.a(usb_prop.o) (Standard_GetDescriptorData)
lib/libusb.a(usb_init.o)      obj/mod_console.o (USB_Init)
lib/libusb.a(usb_int.o)       lib/libusb.a(usb_istr.o) (CTR_LP)
lib/libusb.a(usb_mem.o)       lib/libusb.a(hw_config.o) (UserToPMABufferCopy)
lib/libusb.a(usb_regs.o)      lib/libusb.a(usb_prop.o) (SetBTABLE)
lib/libusb.a(usb_sil.o)       lib/libusb.a(usb_prop.o) (USB_SIL_Init)
lib/libfreertos.a(tasks.o)    /tmp/cc9a8G1U.o (xTaskCreate)
lib/libfreertos.a(list.o)     lib/libfreertos.a(tasks.o) (vListInitialise)
lib/libfreertos.a(queue.o)    obj/mod_auxiliary.o (xQueueGenericSend)
lib/libfreertos.a(timers.o)   lib/libfreertos.a(tasks.o) (xTimerCreateTimerTask)
lib/libfreertos.a(port.o)     lib/libfreertos.a(tasks.o) (pxPortInitialiseStack)
lib/libfreertos.a(heap_4.o)   /tmp/ccrbnG3G.o (pvPortMalloc)
lib/libstm32.a(misc.o)        /tmp/cc9a8G1U.o (NVIC_PriorityGroupConfig)
lib/libstm32.a(stm32f10x_rcc.o)
                              obj/mod_lcd.o (RCC_DeInit)
lib/libstm32.a(stm32f10x_gpio.o)
                              obj/mod_led.o (GPIO_Init)
lib/libstm32.a(stm32f10x_exti.o)
                              obj/mod_sensors.o (EXTI_Init)
lib/libstm32.a(stm32f10x_tim.o)
                              obj/mod_led.o (TIM_TimeBaseInit)
lib/libstm32.a(stm32f10x_adc.o)
                              obj/mod_sensors.o (ADC_Init)
lib/libstm32.a(stm32f10x_usart.o)
                              obj/mod_radio.o (USART_Init)
lib/libstm32.a(stm32f10x_pwr.o)
                              obj/mod_sensors.o (PWR_PVDCmd)
lib/libstm32.a(stm32f10x_dma.o)
                              obj/mod_sensors.o (DMA_DeInit)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o)
                              obj/mod_orientsensor.o (atan2)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o) (__ieee754_atan2)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o) (atan)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_fabs.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o) (fabs)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldf3.o)
                              obj/mod_sensors.o (__aeabi_dmul)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_mulsf3.o)
                              obj/mod_sensors.o (__aeabi_fmul)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubdf3.o)
                              obj/mod_sensors.o (__aeabi_dsub)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivdf3.o)
                              obj/mod_sensors.o (__aeabi_ddiv)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpdf2.o)
                              obj/mod_orientsensor.o (__aeabi_dcmpeq)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixunsdfsi.o)
                              obj/mod_sensors.o (__aeabi_d2uiz)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubsf3.o)
                              obj/mod_sensors.o (__aeabi_ui2f)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivsf3.o)
                              obj/mod_sensors.o (__aeabi_fdiv)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpsf2.o)
                              obj/mod_orientsensor.o (__aeabi_fcmple)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
                              obj/mod_radio.o (atoi)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
                              obj/mod_radio.o (atol)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-errno.o)
                              /tmp/ccTyJAXN.o (__errno)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o (exit)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
                              obj/mod_console.o (fflush)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fileno.o)
                              obj/mod_console.o (fileno)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o) (__sinit)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o) (_fwalk)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
                              obj/mod_console.o (getchar)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o) (_global_impure_ptr)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-init.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o (__libc_init_array)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o) (__retarget_lock_init_recursive)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memcpy-stub.o)
                              obj/mod_auxiliary.o (memcpy)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memset.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o (memset)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
                              (_printf_float)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o) (_printf_common)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
                              obj/mod_sound.o (printf)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
                              lib/libfreertos.a(tasks.o) (_reclaim_reent)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
                              obj/mod_console.o (scanf)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o)
                              obj/mod_console.o (setvbuf)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
                              obj/mod_lcd.o (snprintf)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o) (__sread)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcat.o)
                              obj/mod_radio.o (strcat)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strchr.o)
                              obj/mod_radio.o (strchr)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strlen.o)
                              obj/mod_lcd.o (strlen)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncmp.o)
                              obj/mod_radio.o (strncmp)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncpy.o)
                              obj/mod_lcd.o (strncpy)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o) (_strtol_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-syswrite.o)
                              obj/mod_console.o (write)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-writer.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o) (_write_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-closer.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o) (_close_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ctype_.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o) (_ctype_)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o) (_dtoa_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o) (_getc_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o) (_localeconv_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lseekr.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o) (_lseek_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o) (__swhatbuf_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o) (malloc)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memchr-stub.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o) (memchr)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o) (_Balloc)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o) (_svfprintf_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o) (_vfprintf_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o) (_vfscanf_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o) (_scanf_chars)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-readr.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o) (_read_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o) (__srefill_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o) (__srget_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-sccl.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o) (__sccl)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o) (_strtoul_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o) (_ungetc_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o) (__swbuf_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wsetup.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o) (__swsetup_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o) (__assert_func)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o) (fiprintf)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fstatr.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o) (_fstat_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fvwrite.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o) (__sfvwrite_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-isattyr.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o) (_isatty_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o) (__global_locale)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o) (__ascii_mbtowc)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memmove.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o) (memmove)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcmp.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o) (strcmp)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o) (__ascii_wctomb)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o) (abort)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o) (raise)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o) (_kill_r)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_unorddf2.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o) (__aeabi_dcmpun)
/home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixdfsi.o)
                              /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o) (__aeabi_d2iz)

Discarded input sections

 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crti.o
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crti.o
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crti.o
 .data          0x0000000000000000        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
 .rodata        0x0000000000000000       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
 .text          0x0000000000000000       0x7c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o
 .ARM.extab     0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o
 .ARM.exidx     0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o
 .ARM.attributes
                0x0000000000000000       0x1b /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o
 .text          0x0000000000000000        0x0 obj/mod_led.o
 .data          0x0000000000000000        0x0 obj/mod_led.o
 .bss           0x0000000000000000        0x0 obj/mod_led.o
 .text          0x0000000000000000        0x0 obj/mod_sound.o
 .data          0x0000000000000000        0x0 obj/mod_sound.o
 .bss           0x0000000000000000        0x0 obj/mod_sound.o
 .text.vSound_Signal_Control
                0x0000000000000000       0x58 obj/mod_sound.o
 .text          0x0000000000000000        0x0 obj/mod_sensors.o
 .data          0x0000000000000000        0x0 obj/mod_sensors.o
 .bss           0x0000000000000000        0x0 obj/mod_sensors.o
 .text.get_front_proximity_estimation
                0x0000000000000000       0x2c obj/mod_sensors.o
 .text.trigger_all_dist_sensors
                0x0000000000000000       0x14 obj/mod_sensors.o
 .text.auxil_get_ticks_duration_ms
                0x0000000000000000       0x28 obj/mod_sensors.o
 .bss.SensorsMeasurements
                0x0000000000000000       0x10 obj/mod_sensors.o
 .text          0x0000000000000000        0x0 obj/mod_drive.o
 .data          0x0000000000000000        0x0 obj/mod_drive.o
 .bss           0x0000000000000000        0x0 obj/mod_drive.o
 .text.vExec_lighting_update
                0x0000000000000000       0x2c obj/mod_drive.o
 .text.vExec_signal_update
                0x0000000000000000       0x2c obj/mod_drive.o
 .text.vExec_drive_update
                0x0000000000000000        0x4 obj/mod_drive.o
 .text          0x0000000000000000        0x0 obj/mod_orientsensor.o
 .data          0x0000000000000000        0x0 obj/mod_orientsensor.o
 .bss           0x0000000000000000        0x0 obj/mod_orientsensor.o
 .text.is_robot_stopped
                0x0000000000000000        0x4 obj/mod_orientsensor.o
 .text.ADXL_getDataRate
                0x0000000000000000       0x38 obj/mod_orientsensor.o
 .text.sensor_update_yaw
                0x0000000000000000       0x78 obj/mod_orientsensor.o
 .text.sensor_update_pitch
                0x0000000000000000       0x78 obj/mod_orientsensor.o
 .text.sensor_update_roll
                0x0000000000000000       0x78 obj/mod_orientsensor.o
 .text.sensor_update_acclx
                0x0000000000000000       0xa8 obj/mod_orientsensor.o
 .text.sensor_get_acclx
                0x0000000000000000        0xc obj/mod_orientsensor.o
 .text.sensor_update_accly
                0x0000000000000000       0xb8 obj/mod_orientsensor.o
 .text.sensor_get_accly
                0x0000000000000000        0xc obj/mod_orientsensor.o
 .text.sensor_update_acclz
                0x0000000000000000       0xf0 obj/mod_orientsensor.o
 .text.sensor_get_acclz
                0x0000000000000000        0xc obj/mod_orientsensor.o
 .text.sensor_set_events
                0x0000000000000000       0x20 obj/mod_orientsensor.o
 .text.sensor_get_events
                0x0000000000000000        0xc obj/mod_orientsensor.o
 .text.sensor_get_minor_collision_event
                0x0000000000000000       0x10 obj/mod_orientsensor.o
 .text.sensor_get_major_collision_event
                0x0000000000000000       0x10 obj/mod_orientsensor.o
 .text          0x0000000000000000        0x0 obj/mod_lcd.o
 .data          0x0000000000000000        0x0 obj/mod_lcd.o
 .bss           0x0000000000000000        0x0 obj/mod_lcd.o
 .text          0x0000000000000000        0x0 obj/mod_lighting.o
 .data          0x0000000000000000        0x0 obj/mod_lighting.o
 .bss           0x0000000000000000        0x0 obj/mod_lighting.o
 .text.vLighting_Control
                0x0000000000000000       0x70 obj/mod_lighting.o
 .rodata.vLighting_RF_Control.str1.4
                0x0000000000000000       0x24 obj/mod_lighting.o
 .text.vLighting_RF_Control
                0x0000000000000000       0x78 obj/mod_lighting.o
 .rodata.lighting_leds.1
                0x0000000000000000       0x28 obj/mod_lighting.o
 .text          0x0000000000000000        0x0 obj/mod_console.o
 .data          0x0000000000000000        0x0 obj/mod_console.o
 .bss           0x0000000000000000        0x0 obj/mod_console.o
 .text          0x0000000000000000        0x0 obj/mod_i2c.o
 .data          0x0000000000000000        0x0 obj/mod_i2c.o
 .bss           0x0000000000000000        0x0 obj/mod_i2c.o
 .text          0x0000000000000000        0x0 obj/mod_radio.o
 .data          0x0000000000000000        0x0 obj/mod_radio.o
 .bss           0x0000000000000000        0x0 obj/mod_radio.o
 .text.radio_send_status_frame
                0x0000000000000000       0x4c obj/mod_radio.o
 .text          0x0000000000000000        0x0 obj/mod_auxiliary.o
 .data          0x0000000000000000        0x0 obj/mod_auxiliary.o
 .bss           0x0000000000000000        0x0 obj/mod_auxiliary.o
 .text.manage_state_info
                0x0000000000000000       0x9c obj/mod_auxiliary.o
 .text.light_get_status
                0x0000000000000000       0x44 obj/mod_auxiliary.o
 .text.light_set_status
                0x0000000000000000       0x58 obj/mod_auxiliary.o
 .text.signal_get_state
                0x0000000000000000       0x48 obj/mod_auxiliary.o
 .text          0x0000000000000000        0x0 /tmp/ccPh1D7g.o
 .data          0x0000000000000000        0x0 /tmp/ccPh1D7g.o
 .bss           0x0000000000000000        0x0 /tmp/ccPh1D7g.o
 .text.SystemCoreClockUpdate
                0x0000000000000000       0x5c /tmp/ccPh1D7g.o
 .data.AHBPrescTable
                0x0000000000000000       0x10 /tmp/ccPh1D7g.o
 .data.SystemCoreClock
                0x0000000000000000        0x4 /tmp/ccPh1D7g.o
 .text          0x0000000000000000        0x0 /tmp/cchESTC9.o
 .data          0x0000000000000000        0x0 /tmp/cchESTC9.o
 .bss           0x0000000000000000        0x0 /tmp/cchESTC9.o
 .text          0x0000000000000000        0x0 /tmp/cc9ErQe2.o
 .data          0x0000000000000000        0x0 /tmp/cc9ErQe2.o
 .bss           0x0000000000000000        0x0 /tmp/cc9ErQe2.o
 .text.__get_PSP
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__set_PSP
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__get_MSP
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__get_BASEPRI
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__set_BASEPRI
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__get_PRIMASK
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__set_PRIMASK
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__get_FAULTMASK
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__set_FAULTMASK
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__get_CONTROL
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__set_CONTROL
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__REV    0x0000000000000000        0x4 /tmp/cc9ErQe2.o
 .text.__REV16  0x0000000000000000        0x4 /tmp/cc9ErQe2.o
 .text.__REVSH  0x0000000000000000        0x4 /tmp/cc9ErQe2.o
 .text.__RBIT   0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__LDREXB
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__LDREXH
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__LDREXW
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__STREXB
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__STREXH
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text.__STREXW
                0x0000000000000000        0x8 /tmp/cc9ErQe2.o
 .text          0x0000000000000000        0x0 /tmp/cc9a8G1U.o
 .data          0x0000000000000000        0x0 /tmp/cc9a8G1U.o
 .bss           0x0000000000000000        0x0 /tmp/cc9a8G1U.o
 .text          0x0000000000000000        0x0 /tmp/ccTyJAXN.o
 .data          0x0000000000000000        0x0 /tmp/ccTyJAXN.o
 .bss           0x0000000000000000        0x0 /tmp/ccTyJAXN.o
 .text.initialise_monitor_handles
                0x0000000000000000        0x4 /tmp/ccTyJAXN.o
 .text._open    0x0000000000000000        0xc /tmp/ccTyJAXN.o
 .text._wait    0x0000000000000000       0x14 /tmp/ccTyJAXN.o
 .text._unlink  0x0000000000000000       0x14 /tmp/ccTyJAXN.o
 .text._times   0x0000000000000000        0x8 /tmp/ccTyJAXN.o
 .text._stat    0x0000000000000000        0xc /tmp/ccTyJAXN.o
 .text._link    0x0000000000000000       0x14 /tmp/ccTyJAXN.o
 .text._fork    0x0000000000000000       0x14 /tmp/ccTyJAXN.o
 .text._execve  0x0000000000000000       0x14 /tmp/ccTyJAXN.o
 .bss.__env     0x0000000000000000        0x4 /tmp/ccTyJAXN.o
 .data.environ  0x0000000000000000        0x4 /tmp/ccTyJAXN.o
 .text          0x0000000000000000        0x0 /tmp/ccrbnG3G.o
 .data          0x0000000000000000        0x0 /tmp/ccrbnG3G.o
 .bss           0x0000000000000000        0x0 /tmp/ccrbnG3G.o
 .text._sbrk    0x0000000000000000       0x14 /tmp/ccrbnG3G.o
 .text.__malloc_lock
                0x0000000000000000        0x4 /tmp/ccrbnG3G.o
 .text.__malloc_unlock
                0x0000000000000000        0x4 /tmp/ccrbnG3G.o
 .text.__env_lock
                0x0000000000000000        0x4 /tmp/ccrbnG3G.o
 .text.__env_unlock
                0x0000000000000000        0x4 /tmp/ccrbnG3G.o
 .text          0x0000000000000000       0x14 /tmp/ccJdFGaA.o
 .data          0x0000000000000000        0x0 /tmp/ccJdFGaA.o
 .bss           0x0000000000000000        0x0 /tmp/ccJdFGaA.o
 .text          0x0000000000000000        0x0 lib/libusb.a(hw_config.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(hw_config.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(hw_config.o)
 .text.Enter_LowPowerMode
                0x0000000000000000        0xc lib/libusb.a(hw_config.o)
 .text.Leave_LowPowerMode
                0x0000000000000000       0x20 lib/libusb.a(hw_config.o)
 .text.USB_Interrupts_Config
                0x0000000000000000       0x30 lib/libusb.a(hw_config.o)
 .text.USB_Send_Data
                0x0000000000000000       0x4c lib/libusb.a(hw_config.o)
 .text.USB_data_interpreter
                0x0000000000000000        0x4 lib/libusb.a(hw_config.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_desc.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_desc.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_desc.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_endp.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_endp.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_endp.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_istr.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_istr.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_istr.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_prop.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_prop.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_prop.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_pwr.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_pwr.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_pwr.o)
 .text.PowerOff
                0x0000000000000000       0x24 lib/libusb.a(usb_pwr.o)
 .text.Suspend  0x0000000000000000       0x28 lib/libusb.a(usb_pwr.o)
 .text.Resume_Init
                0x0000000000000000       0x24 lib/libusb.a(usb_pwr.o)
 .text.Resume   0x0000000000000000       0xc8 lib/libusb.a(usb_pwr.o)
 .bss.ResumeS   0x0000000000000000        0x2 lib/libusb.a(usb_pwr.o)
 .data.fSuspendEnabled
                0x0000000000000000        0x1 lib/libusb.a(usb_pwr.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_core.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_core.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_core.o)
 .text.Standard_SetDeviceFeature
                0x0000000000000000       0x24 lib/libusb.a(usb_core.o)
 .text.Post0_Process
                0x0000000000000000       0x44 lib/libusb.a(usb_core.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_init.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_init.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_init.o)
 .bss.SaveState
                0x0000000000000000        0x2 lib/libusb.a(usb_init.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_int.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_int.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_int.o)
 .text.CTR_HP   0x0000000000000000       0xa0 lib/libusb.a(usb_int.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_mem.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_mem.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_mem.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_regs.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_regs.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_regs.o)
 .text.SetCNTR  0x0000000000000000        0xc lib/libusb.a(usb_regs.o)
 .text.GetCNTR  0x0000000000000000       0x10 lib/libusb.a(usb_regs.o)
 .text.SetISTR  0x0000000000000000        0xc lib/libusb.a(usb_regs.o)
 .text.GetISTR  0x0000000000000000       0x10 lib/libusb.a(usb_regs.o)
 .text.GetFNR   0x0000000000000000       0x10 lib/libusb.a(usb_regs.o)
 .text.SetDADDR
                0x0000000000000000        0xc lib/libusb.a(usb_regs.o)
 .text.GetDADDR
                0x0000000000000000       0x10 lib/libusb.a(usb_regs.o)
 .text.GetBTABLE
                0x0000000000000000       0x10 lib/libusb.a(usb_regs.o)
 .text.SetENDPOINT
                0x0000000000000000       0x10 lib/libusb.a(usb_regs.o)
 .text.GetENDPOINT
                0x0000000000000000       0x10 lib/libusb.a(usb_regs.o)
 .text.GetEPType
                0x0000000000000000       0x14 lib/libusb.a(usb_regs.o)
 .text.SetDouBleBuffEPStall
                0x0000000000000000       0x2c lib/libusb.a(usb_regs.o)
 .text.GetEPTxStatus
                0x0000000000000000       0x14 lib/libusb.a(usb_regs.o)
 .text.GetEPRxStatus
                0x0000000000000000       0x14 lib/libusb.a(usb_regs.o)
 .text.SetEP_KIND
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.ClearEP_KIND
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.Set_Status_Out
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.SetEPDoubleBuff
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.ClearEPDoubleBuff
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.GetTxStallStatus
                0x0000000000000000       0x1c lib/libusb.a(usb_regs.o)
 .text.GetRxStallStatus
                0x0000000000000000       0x1c lib/libusb.a(usb_regs.o)
 .text.ClearEP_CTR_RX
                0x0000000000000000       0x18 lib/libusb.a(usb_regs.o)
 .text.ClearEP_CTR_TX
                0x0000000000000000       0x18 lib/libusb.a(usb_regs.o)
 .text.ToggleDTOG_RX
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.ToggleDTOG_TX
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.SetEPAddress
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.GetEPAddress
                0x0000000000000000       0x14 lib/libusb.a(usb_regs.o)
 .text.SetEPCountRxReg
                0x0000000000000000       0x28 lib/libusb.a(usb_regs.o)
 .text.GetEPTxCount
                0x0000000000000000       0x24 lib/libusb.a(usb_regs.o)
 .text.SetEPDblBuffAddr
                0x0000000000000000       0x44 lib/libusb.a(usb_regs.o)
 .text.SetEPDblBuf0Addr
                0x0000000000000000       0x24 lib/libusb.a(usb_regs.o)
 .text.SetEPDblBuf1Addr
                0x0000000000000000       0x24 lib/libusb.a(usb_regs.o)
 .text.GetEPDblBuf0Addr
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.GetEPDblBuf1Addr
                0x0000000000000000       0x20 lib/libusb.a(usb_regs.o)
 .text.SetEPDblBuffCount
                0x0000000000000000       0xb4 lib/libusb.a(usb_regs.o)
 .text.SetEPDblBuf0Count
                0x0000000000000000       0x64 lib/libusb.a(usb_regs.o)
 .text.SetEPDblBuf1Count
                0x0000000000000000       0x64 lib/libusb.a(usb_regs.o)
 .text.GetEPDblBuf0Count
                0x0000000000000000       0x24 lib/libusb.a(usb_regs.o)
 .text.GetEPDblBuf1Count
                0x0000000000000000       0x24 lib/libusb.a(usb_regs.o)
 .text.GetEPDblBufDir
                0x0000000000000000       0x4c lib/libusb.a(usb_regs.o)
 .text.FreeUserBuffer
                0x0000000000000000       0x48 lib/libusb.a(usb_regs.o)
 .text.ToWord   0x0000000000000000        0x8 lib/libusb.a(usb_regs.o)
 .text          0x0000000000000000        0x0 lib/libusb.a(usb_sil.o)
 .data          0x0000000000000000        0x0 lib/libusb.a(usb_sil.o)
 .bss           0x0000000000000000        0x0 lib/libusb.a(usb_sil.o)
 .text.USB_SIL_Write
                0x0000000000000000       0x28 lib/libusb.a(usb_sil.o)
 .text          0x0000000000000000        0x0 lib/libfreertos.a(tasks.o)
 .data          0x0000000000000000        0x0 lib/libfreertos.a(tasks.o)
 .bss           0x0000000000000000        0x0 lib/libfreertos.a(tasks.o)
 .text.vTaskDelete
                0x0000000000000000       0xf4 lib/libfreertos.a(tasks.o)
 .text.eTaskGetState
                0x0000000000000000       0x68 lib/libfreertos.a(tasks.o)
 .text.uxTaskPriorityGetFromISR
                0x0000000000000000       0x30 lib/libfreertos.a(tasks.o)
 .text.vTaskResume
                0x0000000000000000       0x8c lib/libfreertos.a(tasks.o)
 .text.xTaskResumeFromISR
                0x0000000000000000       0xa4 lib/libfreertos.a(tasks.o)
 .text.vTaskEndScheduler
                0x0000000000000000       0x20 lib/libfreertos.a(tasks.o)
 .text.xTaskGetTickCountFromISR
                0x0000000000000000        0xc lib/libfreertos.a(tasks.o)
 .text.uxTaskGetNumberOfTasks
                0x0000000000000000        0xc lib/libfreertos.a(tasks.o)
 .text.pcTaskGetName
                0x0000000000000000       0x14 lib/libfreertos.a(tasks.o)
 .text.vTaskSuspend
                0x0000000000000000       0xe8 lib/libfreertos.a(tasks.o)
 .text.vTaskPlaceOnUnorderedEventList
                0x0000000000000000       0x28 lib/libfreertos.a(tasks.o)
 .text.vTaskRemoveFromUnorderedEventList
                0x0000000000000000       0x58 lib/libfreertos.a(tasks.o)
 .text.vTaskSetTimeOutState
                0x0000000000000000       0x24 lib/libfreertos.a(tasks.o)
 .text.uxTaskGetTaskNumber
                0x0000000000000000        0x8 lib/libfreertos.a(tasks.o)
 .text.vTaskSetTaskNumber
                0x0000000000000000        0x8 lib/libfreertos.a(tasks.o)
 .text.vTaskGetInfo
                0x0000000000000000       0xf0 lib/libfreertos.a(tasks.o)
 .text.prvListTasksWithinSingleList.part.0
                0x0000000000000000       0x50 lib/libfreertos.a(tasks.o)
 .text.uxTaskGetSystemState
                0x0000000000000000       0xe4 lib/libfreertos.a(tasks.o)
 .text.xTaskGetCurrentTaskHandle
                0x0000000000000000        0xc lib/libfreertos.a(tasks.o)
 .text.uxTaskResetEventItemValue
                0x0000000000000000       0x18 lib/libfreertos.a(tasks.o)
 .text.ulTaskNotifyTake
                0x0000000000000000       0x74 lib/libfreertos.a(tasks.o)
 .text.xTaskGenericNotify
                0x0000000000000000       0xc0 lib/libfreertos.a(tasks.o)
 .text.vTaskNotifyGiveFromISR
                0x0000000000000000       0xb4 lib/libfreertos.a(tasks.o)
 .text.xTaskNotifyStateClear
                0x0000000000000000       0x30 lib/libfreertos.a(tasks.o)
 .text          0x0000000000000000        0x0 lib/libfreertos.a(list.o)
 .data          0x0000000000000000        0x0 lib/libfreertos.a(list.o)
 .bss           0x0000000000000000        0x0 lib/libfreertos.a(list.o)
 .text          0x0000000000000000        0x0 lib/libfreertos.a(queue.o)
 .data          0x0000000000000000        0x0 lib/libfreertos.a(queue.o)
 .bss           0x0000000000000000        0x0 lib/libfreertos.a(queue.o)
 .text.xQueueGenericReset
                0x0000000000000000       0x7c lib/libfreertos.a(queue.o)
 .text.xQueueCreateCountingSemaphore
                0x0000000000000000       0x60 lib/libfreertos.a(queue.o)
 .text.xQueueGiveMutexRecursive
                0x0000000000000000       0x2c lib/libfreertos.a(queue.o)
 .text.xQueueGiveFromISR
                0x0000000000000000       0x78 lib/libfreertos.a(queue.o)
 .text.xQueueTakeMutexRecursive
                0x0000000000000000       0x2c lib/libfreertos.a(queue.o)
 .text.xQueuePeek
                0x0000000000000000      0x120 lib/libfreertos.a(queue.o)
 .text.xQueueReceiveFromISR
                0x0000000000000000       0x90 lib/libfreertos.a(queue.o)
 .text.xQueuePeekFromISR
                0x0000000000000000       0x48 lib/libfreertos.a(queue.o)
 .text.uxQueueMessagesWaiting
                0x0000000000000000       0x14 lib/libfreertos.a(queue.o)
 .text.uxQueueSpacesAvailable
                0x0000000000000000       0x18 lib/libfreertos.a(queue.o)
 .text.uxQueueMessagesWaitingFromISR
                0x0000000000000000        0x4 lib/libfreertos.a(queue.o)
 .text.vQueueDelete
                0x0000000000000000        0x4 lib/libfreertos.a(queue.o)
 .text.uxQueueGetQueueNumber
                0x0000000000000000        0x4 lib/libfreertos.a(queue.o)
 .text.vQueueSetQueueNumber
                0x0000000000000000        0x4 lib/libfreertos.a(queue.o)
 .text.ucQueueGetQueueType
                0x0000000000000000        0x8 lib/libfreertos.a(queue.o)
 .text.xQueueIsQueueEmptyFromISR
                0x0000000000000000        0xc lib/libfreertos.a(queue.o)
 .text.xQueueIsQueueFullFromISR
                0x0000000000000000       0x10 lib/libfreertos.a(queue.o)
 .text          0x0000000000000000        0x0 lib/libfreertos.a(timers.o)
 .data          0x0000000000000000        0x0 lib/libfreertos.a(timers.o)
 .bss           0x0000000000000000        0x0 lib/libfreertos.a(timers.o)
 .text.xTimerGetTimerDaemonTaskHandle
                0x0000000000000000        0xc lib/libfreertos.a(timers.o)
 .text.xTimerGetPeriod
                0x0000000000000000        0x4 lib/libfreertos.a(timers.o)
 .text.xTimerGetExpiryTime
                0x0000000000000000        0x4 lib/libfreertos.a(timers.o)
 .text.pcTimerGetName
                0x0000000000000000        0x4 lib/libfreertos.a(timers.o)
 .text.xTimerIsTimerActive
                0x0000000000000000       0x18 lib/libfreertos.a(timers.o)
 .text.pvTimerGetTimerID
                0x0000000000000000       0x14 lib/libfreertos.a(timers.o)
 .text.vTimerSetTimerID
                0x0000000000000000       0x14 lib/libfreertos.a(timers.o)
 .text.uxTimerGetTimerNumber
                0x0000000000000000        0x4 lib/libfreertos.a(timers.o)
 .text.vTimerSetTimerNumber
                0x0000000000000000        0x4 lib/libfreertos.a(timers.o)
 .text          0x0000000000000000        0x0 lib/libfreertos.a(port.o)
 .data          0x0000000000000000        0x0 lib/libfreertos.a(port.o)
 .bss           0x0000000000000000        0x0 lib/libfreertos.a(port.o)
 .text.vPortEndScheduler
                0x0000000000000000        0x4 lib/libfreertos.a(port.o)
 .text          0x0000000000000000        0x0 lib/libfreertos.a(heap_4.o)
 .data          0x0000000000000000        0x0 lib/libfreertos.a(heap_4.o)
 .bss           0x0000000000000000        0x0 lib/libfreertos.a(heap_4.o)
 .text.xPortGetFreeHeapSize
                0x0000000000000000        0xc lib/libfreertos.a(heap_4.o)
 .text.xPortGetMinimumEverFreeHeapSize
                0x0000000000000000        0xc lib/libfreertos.a(heap_4.o)
 .text.vPortInitialiseBlocks
                0x0000000000000000        0x4 lib/libfreertos.a(heap_4.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(misc.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(misc.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(misc.o)
 .text.NVIC_SetVectorTable
                0x0000000000000000       0x14 lib/libstm32.a(misc.o)
 .text.NVIC_SystemLPConfig
                0x0000000000000000       0x1c lib/libstm32.a(misc.o)
 .text.SysTick_CLKSourceConfig
                0x0000000000000000       0x18 lib/libstm32.a(misc.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_rcc.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_rcc.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_HSEConfig
                0x0000000000000000       0x38 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_WaitForHSEStartUp
                0x0000000000000000       0x30 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_AdjustHSICalibrationValue
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_HSICmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_PLLConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_PLLCmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_SYSCLKConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_GetSYSCLKSource
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_HCLKConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_PCLK1Config
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_PCLK2Config
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_ITConfig
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_ADCCLKConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_LSEConfig
                0x0000000000000000       0x28 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_LSICmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_RTCCLKConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_RTCCLKCmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_APB1PeriphResetCmd
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_BackupResetCmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_ClockSecuritySystemCmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_MCOConfig
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_ClearFlag
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_GetITStatus
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_rcc.o)
 .text.RCC_ClearITPendingBit
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_rcc.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_gpio.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_gpio.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_DeInit
                0x0000000000000000       0xdc lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_AFIODeInit
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_StructInit
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_ReadInputData
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_ReadOutputData
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_Write
                0x0000000000000000        0x4 lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_PinLockConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_EventOutputConfig
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_EventOutputCmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_EXTILineConfig
                0x0000000000000000       0x2c lib/libstm32.a(stm32f10x_gpio.o)
 .text.GPIO_ETH_MediaInterfaceConfig
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_gpio.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_exti.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_exti.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_exti.o)
 .text.EXTI_DeInit
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_exti.o)
 .text.EXTI_StructInit
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_exti.o)
 .text.EXTI_GenerateSWInterrupt
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_exti.o)
 .text.EXTI_GetFlagStatus
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_exti.o)
 .text.EXTI_ClearFlag
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_exti.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_tim.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_tim.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_DeInit
                0x0000000000000000      0x234 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC4Init
                0x0000000000000000       0x78 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_PWMIConfig
                0x0000000000000000       0xa8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_BDTRConfig
                0x0000000000000000       0x30 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_TimeBaseStructInit
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OCStructInit
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ICStructInit
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_BDTRStructInit
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_GenerateEvent
                0x0000000000000000        0x4 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_DMAConfig
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_DMACmd
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_InternalClockConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ITRxExternalClockConfig
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_TIxExternalClockConfig
                0x0000000000000000       0x9c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ETRClockMode1Config
                0x0000000000000000       0x24 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ETRClockMode2Config
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ETRConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_PrescalerConfig
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_CounterModeConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_EncoderInterfaceConfig
                0x0000000000000000       0x50 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ForcedOC1Config
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ForcedOC2Config
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ForcedOC3Config
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ForcedOC4Config
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ARRPreloadConfig
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SelectCOM
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SelectCCDMA
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_CCPreloadControl
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC2PreloadConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC4PreloadConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC1FastConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC2FastConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC3FastConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC4FastConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ClearOC1Ref
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ClearOC2Ref
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ClearOC3Ref
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ClearOC4Ref
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC1NPolarityConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC2PolarityConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC2NPolarityConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC3NPolarityConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_OC4PolarityConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_CCxCmd
                0x0000000000000000       0x20 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_CCxNCmd
                0x0000000000000000       0x20 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SelectOCxM
                0x0000000000000000       0x60 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_UpdateDisableConfig
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_UpdateRequestConfig
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SelectHallSensor
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SelectOnePulseMode
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SelectOutputTrigger
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SelectMasterSlaveMode
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetAutoreload
                0x0000000000000000        0x4 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetCompare2
                0x0000000000000000        0x4 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetCompare4
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetIC1Prescaler
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetIC2Prescaler
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetIC3Prescaler
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetIC4Prescaler
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_SetClockDivision
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_GetCapture1
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_GetCapture2
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_GetCapture3
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_GetCapture4
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_GetPrescaler
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_GetFlagStatus
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_ClearFlag
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_tim.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_adc.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_adc.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_DeInit
                0x0000000000000000       0x6c lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_StructInit
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_ITConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_GetSoftwareStartConvStatus
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_DiscModeChannelCountConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_DiscModeCmd
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_GetConversionValue
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_GetDualModeConversionValue
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_AutoInjectedConvCmd
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_InjectedDiscModeCmd
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_ExternalTrigInjectedConvConfig
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_ExternalTrigInjectedConvCmd
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_SoftwareStartInjectedConvCmd
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_GetSoftwareStartInjectedConvCmdStatus
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_InjectedChannelConfig
                0x0000000000000000       0x60 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_InjectedSequencerLengthConfig
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_SetInjectedOffset
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_GetInjectedConversionValue
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_AnalogWatchdogCmd
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_AnalogWatchdogThresholdsConfig
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_AnalogWatchdogSingleChannelConfig
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_GetFlagStatus
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_ClearFlag
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_GetITStatus
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_adc.o)
 .text.ADC_ClearITPendingBit
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_adc.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_usart.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_usart.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_DeInit
                0x0000000000000000       0xb0 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_StructInit
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_ClockInit
                0x0000000000000000       0x28 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_ClockStructInit
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_DMACmd
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_ReceiverWakeUpCmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_LINBreakDetectLengthConfig
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_LINCmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_SendBreak
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_SetGuardTime
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_SetPrescaler
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_SmartCardCmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_SmartCardNACKCmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_HalfDuplexCmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_OverSampling8Cmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_OneBitMethodCmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_IrDAConfig
                0x0000000000000000       0x18 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_IrDACmd
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_GetITStatus
                0x0000000000000000       0x44 lib/libstm32.a(stm32f10x_usart.o)
 .text.USART_ClearITPendingBit
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_usart.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_pwr.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_pwr.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_pwr.o)
 .text.PWR_DeInit
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_pwr.o)
 .text.PWR_BackupAccessCmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_pwr.o)
 .text.PWR_WakeUpPinCmd
                0x0000000000000000        0xc lib/libstm32.a(stm32f10x_pwr.o)
 .text.PWR_EnterSTOPMode
                0x0000000000000000       0x3c lib/libstm32.a(stm32f10x_pwr.o)
 .text.PWR_EnterSTANDBYMode
                0x0000000000000000       0x28 lib/libstm32.a(stm32f10x_pwr.o)
 .text.PWR_ClearFlag
                0x0000000000000000       0x10 lib/libstm32.a(stm32f10x_pwr.o)
 .text          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_dma.o)
 .data          0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_dma.o)
 .bss           0x0000000000000000        0x0 lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_StructInit
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_ITConfig
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_SetCurrDataCounter
                0x0000000000000000        0x4 lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_GetCurrDataCounter
                0x0000000000000000        0x8 lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_GetFlagStatus
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_ClearFlag
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_GetITStatus
                0x0000000000000000       0x1c lib/libstm32.a(stm32f10x_dma.o)
 .text.DMA_ClearITPendingBit
                0x0000000000000000       0x14 lib/libstm32.a(stm32f10x_dma.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_fabs.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_fabs.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_fabs.o)
 .text          0x0000000000000000      0x254 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldf3.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldf3.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldf3.o)
 .debug_frame   0x0000000000000000       0x30 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldf3.o)
 .ARM.attributes
                0x0000000000000000       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldf3.o)
 .text          0x0000000000000000      0x168 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_mulsf3.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_mulsf3.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_mulsf3.o)
 .debug_frame   0x0000000000000000       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_mulsf3.o)
 .ARM.attributes
                0x0000000000000000       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_mulsf3.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubdf3.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubdf3.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivdf3.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivdf3.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpdf2.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpdf2.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixunsdfsi.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixunsdfsi.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubsf3.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubsf3.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivsf3.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivsf3.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpsf2.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpsf2.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
 .text._atoi_r  0x0000000000000000        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
 .text._atol_r  0x0000000000000000        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-errno.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-errno.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-errno.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o)
 .text.exit     0x0000000000000000       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o)
 .debug_frame   0x0000000000000000       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o)
 .ARM.attributes
                0x0000000000000000       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-exit.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fileno.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fileno.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fileno.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .text.__fp_lock
                0x0000000000000000       0x18 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .text.__fp_unlock
                0x0000000000000000       0x18 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .text._cleanup
                0x0000000000000000        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .text.__fp_lock_all
                0x0000000000000000       0x1c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .text.__fp_unlock_all
                0x0000000000000000       0x1c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
 .text._getchar_r
                0x0000000000000000       0x1a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-init.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-init.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-init.o)
 .text.__libc_init_array
                0x0000000000000000       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-init.o)
 .debug_frame   0x0000000000000000       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-init.o)
 .ARM.attributes
                0x0000000000000000       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-init.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text.__retarget_lock_init
                0x0000000000000000        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text.__retarget_lock_close
                0x0000000000000000        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text.__retarget_lock_close_recursive
                0x0000000000000000        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text.__retarget_lock_acquire
                0x0000000000000000        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text.__retarget_lock_try_acquire
                0x0000000000000000        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text.__retarget_lock_try_acquire_recursive
                0x0000000000000000        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text.__retarget_lock_release
                0x0000000000000000        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss.__lock___arc4random_mutex
                0x0000000000000000        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss.__lock___at_quick_exit_mutex
                0x0000000000000000        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss.__lock___atexit_recursive_mutex
                0x0000000000000000        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss.__lock___dd_hash_mutex
                0x0000000000000000        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss.__lock___env_recursive_mutex
                0x0000000000000000        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss.__lock___malloc_recursive_mutex
                0x0000000000000000        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .bss.__lock___tz_mutex
                0x0000000000000000        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memcpy-stub.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memcpy-stub.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memcpy-stub.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memset.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memset.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memset.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
 .text._printf_r
                0x0000000000000000       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
 .text._scanf_r
                0x0000000000000000       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
 .text._snprintf_r
                0x0000000000000000       0x5e /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
 .text.__seofread
                0x0000000000000000        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcat.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcat.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcat.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strchr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strchr.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strchr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strlen.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strlen.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncmp.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncmp.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncmp.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncpy.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncpy.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncpy.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
 .text.strtol_l
                0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-syswrite.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-syswrite.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-syswrite.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-writer.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-writer.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-writer.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-closer.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-closer.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-closer.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ctype_.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ctype_.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ctype_.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
 .text.getc     0x0000000000000000       0x88 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
 .text.__localeconv_l
                0x0000000000000000        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
 .text.localeconv
                0x0000000000000000        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lseekr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lseekr.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lseekr.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
 .text.free     0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memchr-stub.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memchr-stub.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memchr-stub.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text.__s2b    0x0000000000000000       0x94 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text.__ulp    0x0000000000000000       0x4c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text.__b2d    0x0000000000000000       0xa8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text.__ratio  0x0000000000000000       0x5c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text._mprec_log10
                0x0000000000000000       0x38 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text.__copybits
                0x0000000000000000       0x46 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text.__any_on
                0x0000000000000000       0x42 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .rodata.__mprec_tinytens
                0x0000000000000000       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
 .text.__ssprint_r
                0x0000000000000000       0xf6 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .text.__sprint_r
                0x0000000000000000       0x1a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .text.vfprintf
                0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .text.vfscanf  0x0000000000000000       0x5c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .text.__svfscanf
                0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-readr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-readr.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-readr.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
 .text.__srget  0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-sccl.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-sccl.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-sccl.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .text.strtoul_l
                0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .text.strtoul  0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
 .text.ungetc   0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
 .text.__swbuf  0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wsetup.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wsetup.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wsetup.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
 .text.__assert
                0x0000000000000000        0xa /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
 .text._fprintf_r
                0x0000000000000000       0x1a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fstatr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fstatr.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fstatr.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fvwrite.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fvwrite.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fvwrite.o)
 .text.__sfvwrite_r
                0x0000000000000000      0x290 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fvwrite.o)
 .debug_frame   0x0000000000000000       0x3c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fvwrite.o)
 .ARM.attributes
                0x0000000000000000       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fvwrite.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-isattyr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-isattyr.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-isattyr.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .text._setlocale_r
                0x0000000000000000       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .text.__locale_mb_cur_max
                0x0000000000000000        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .text.setlocale
                0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .bss._PathLocale
                0x0000000000000000        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
 .text._mbtowc_r
                0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memmove.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memmove.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memmove.o)
 .text          0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcmp.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcmp.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcmp.o)
 .debug_frame   0x0000000000000000       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcmp.o)
 .ARM.attributes
                0x0000000000000000       0x1b /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcmp.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
 .text._wctomb_r
                0x0000000000000000       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .text._init_signal_r
                0x0000000000000000       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .text._signal_r
                0x0000000000000000       0x30 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .text.__sigtramp_r
                0x0000000000000000       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .text.signal   0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .text._init_signal
                0x0000000000000000        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .text.__sigtramp
                0x0000000000000000       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_unorddf2.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_unorddf2.o)
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixdfsi.o)
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixdfsi.o)
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtend.o
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtend.o
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtend.o
 .rodata        0x0000000000000000       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtend.o
 .eh_frame      0x0000000000000000        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtend.o
 .ARM.attributes
                0x0000000000000000       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtend.o
 .text          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtn.o
 .data          0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtn.o
 .bss           0x0000000000000000        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtn.o

Memory Configuration

Name             Origin             Length             Attributes
FLASH            0x0000000008000000 0x0000000000020000 xr
RAM              0x0000000020000000 0x0000000000005000 xrw
MEMORY_B1        0x0000000060000000 0x0000000000000000 xr
*default*        0x0000000000000000 0xffffffffffffffff

Linker script and memory map

LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crti.o
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/crt0.o
LOAD obj/mod_led.o
LOAD obj/mod_sound.o
LOAD obj/mod_sensors.o
LOAD obj/mod_drive.o
LOAD obj/mod_orientsensor.o
LOAD obj/mod_lcd.o
LOAD obj/mod_lighting.o
LOAD obj/mod_console.o
LOAD obj/mod_i2c.o
LOAD obj/mod_radio.o
LOAD obj/mod_auxiliary.o
LOAD /tmp/ccPh1D7g.o
LOAD /tmp/cchESTC9.o
LOAD /tmp/cc9ErQe2.o
LOAD /tmp/cc9a8G1U.o
LOAD /tmp/ccTyJAXN.o
LOAD /tmp/ccrbnG3G.o
LOAD /tmp/ccJdFGaA.o
LOAD lib/libusb.a
LOAD lib/libfreertos.a
LOAD lib/libstm32.a
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a
START GROUP
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a
END GROUP
START GROUP
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a
END GROUP
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtend.o
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtn.o
                0x0000000020005000                _estack = 0x20005000
                0x0000000000000000                _Min_Heap_Size = 0x0
                0x0000000000000200                _Min_Stack_Size = 0x200
                0x0000000000000000                HEAP_SIZE = _Min_Heap_Size
                0x0000000000000200                STACK_SIZE = _Min_Stack_Size

.isr_vector     0x0000000008000000      0x10c
                0x0000000008000000                . = ALIGN (0x4)
 *(.isr_vector)
 .isr_vector    0x0000000008000000      0x10c /tmp/ccJdFGaA.o
                0x0000000008000000                g_pfnVectors
                0x000000000800010c                . = ALIGN (0x4)

.eh_frame       0x000000000800010c        0x0
 .eh_frame      0x000000000800010c        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o

.text           0x0000000008000110    0x10a2c
                0x0000000008000110                . = ALIGN (0x4)
 *(.text)
 .text          0x0000000008000110       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
 .text          0x0000000008000150      0x378 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubdf3.o)
                0x0000000008000150                __aeabi_drsub
                0x0000000008000158                __aeabi_dsub
                0x0000000008000158                __subdf3
                0x000000000800015c                __adddf3
                0x000000000800015c                __aeabi_dadd
                0x00000000080003d4                __aeabi_ui2d
                0x00000000080003d4                __floatunsidf
                0x00000000080003f4                __floatsidf
                0x00000000080003f4                __aeabi_i2d
                0x0000000008000418                __extendsfdf2
                0x0000000008000418                __aeabi_f2d
                0x000000000800045c                __aeabi_ul2d
                0x000000000800045c                __floatundidf
                0x000000000800046c                __floatdidf
                0x000000000800046c                __aeabi_l2d
 .text          0x00000000080004c8      0x424 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivdf3.o)
                0x00000000080004c8                __aeabi_dmul
                0x00000000080004c8                __muldf3
                0x000000000800071c                __aeabi_ddiv
                0x000000000800071c                __divdf3
 .text          0x00000000080008ec      0x110 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpdf2.o)
                0x00000000080008ec                __gtdf2
                0x00000000080008ec                __gedf2
                0x00000000080008f4                __ltdf2
                0x00000000080008f4                __ledf2
                0x00000000080008fc                __cmpdf2
                0x00000000080008fc                __eqdf2
                0x00000000080008fc                __nedf2
                0x0000000008000978                __aeabi_cdrcmple
                0x0000000008000988                __aeabi_cdcmple
                0x0000000008000988                __aeabi_cdcmpeq
                0x0000000008000998                __aeabi_dcmpeq
                0x00000000080009ac                __aeabi_dcmplt
                0x00000000080009c0                __aeabi_dcmple
                0x00000000080009d4                __aeabi_dcmpge
                0x00000000080009e8                __aeabi_dcmpgt
 .text          0x00000000080009fc       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixunsdfsi.o)
                0x00000000080009fc                __fixunsdfsi
                0x00000000080009fc                __aeabi_d2uiz
 .text          0x0000000008000a3c      0x21c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubsf3.o)
                0x0000000008000a3c                __aeabi_frsub
                0x0000000008000a44                __subsf3
                0x0000000008000a44                __aeabi_fsub
                0x0000000008000a48                __aeabi_fadd
                0x0000000008000a48                __addsf3
                0x0000000008000ba8                __aeabi_ui2f
                0x0000000008000ba8                __floatunsisf
                0x0000000008000bb0                __aeabi_i2f
                0x0000000008000bb0                __floatsisf
                0x0000000008000bcc                __floatundisf
                0x0000000008000bcc                __aeabi_ul2f
                0x0000000008000bdc                __floatdisf
                0x0000000008000bdc                __aeabi_l2f
 .text          0x0000000008000c58      0x2a0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivsf3.o)
                0x0000000008000c58                __mulsf3
                0x0000000008000c58                __aeabi_fmul
                0x0000000008000dc0                __divsf3
                0x0000000008000dc0                __aeabi_fdiv
 .text          0x0000000008000ef8       0xec /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpsf2.o)
                0x0000000008000ef8                __gtsf2
                0x0000000008000ef8                __gesf2
                0x0000000008000f00                __lesf2
                0x0000000008000f00                __ltsf2
                0x0000000008000f08                __nesf2
                0x0000000008000f08                __eqsf2
                0x0000000008000f08                __cmpsf2
                0x0000000008000f68                __aeabi_cfrcmple
                0x0000000008000f70                __aeabi_cfcmple
                0x0000000008000f70                __aeabi_cfcmpeq
                0x0000000008000f80                __aeabi_fcmpeq
                0x0000000008000f94                __aeabi_fcmplt
                0x0000000008000fa8                __aeabi_fcmple
                0x0000000008000fbc                __aeabi_fcmpge
                0x0000000008000fd0                __aeabi_fcmpgt
 .text          0x0000000008000fe4       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strlen.o)
                0x0000000008000fe4                strlen
 .text          0x0000000008000ff4       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_unorddf2.o)
                0x0000000008000ff4                __aeabi_dcmpun
                0x0000000008000ff4                __unorddf2
 .text          0x0000000008001020       0x50 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixdfsi.o)
                0x0000000008001020                __fixdfsi
                0x0000000008001020                __aeabi_d2iz
 *(.text*)
 .text.led_update_pwm_duty
                0x0000000008001070       0x14 obj/mod_led.o
                0x0000000008001070                led_update_pwm_duty
 .text.led_set_brightness_level
                0x0000000008001084       0x48 obj/mod_led.o
                0x0000000008001084                led_set_brightness_level
 .text.vLED_Configuration
                0x00000000080010cc       0xc4 obj/mod_led.o
                0x00000000080010cc                vLED_Configuration
 .text.vLedTask
                0x0000000008001190       0x50 obj/mod_led.o
                0x0000000008001190                vLedTask
 .text.vSound_configuration
                0x00000000080011e0       0x38 obj/mod_sound.o
                0x00000000080011e0                vSound_configuration
 .text.vSound_Signal_RF_Control
                0x0000000008001218       0x2c obj/mod_sound.o
                0x0000000008001218                vSound_Signal_RF_Control
 .text.vSound_is_signal
                0x0000000008001244       0x18 obj/mod_sound.o
                0x0000000008001244                vSound_is_signal
 .text.get_front_distance
                0x000000000800125c       0x40 obj/mod_sensors.o
                0x000000000800125c                get_front_distance
 .text.get_back_distance
                0x000000000800129c       0x40 obj/mod_sensors.o
                0x000000000800129c                get_back_distance
 .text.get_battery_status
                0x00000000080012dc       0x20 obj/mod_sensors.o
                0x00000000080012dc                get_battery_status
 *fill*         0x00000000080012fc        0x4 
 .text.get_internal_temp
                0x0000000008001300       0x70 obj/mod_sensors.o
                0x0000000008001300                get_internal_temp
 .text.get_illumination
                0x0000000008001370       0x58 obj/mod_sensors.o
                0x0000000008001370                get_illumination
 .text.trigger_dist_sensor
                0x00000000080013c8       0x8c obj/mod_sensors.o
                0x00000000080013c8                trigger_dist_sensor
 .text.PWR_PVDLevelGet
                0x0000000008001454       0x10 obj/mod_sensors.o
                0x0000000008001454                PWR_PVDLevelGet
 .text.vCheckSupplyVoltage
                0x0000000008001464       0x64 obj/mod_sensors.o
                0x0000000008001464                vCheckSupplyVoltage
 .text.vAnalogSensors_config_gpio
                0x00000000080014c8       0x30 obj/mod_sensors.o
                0x00000000080014c8                vAnalogSensors_config_gpio
 .text.vAnalogSensors_config_timer
                0x00000000080014f8       0x94 obj/mod_sensors.o
                0x00000000080014f8                vAnalogSensors_config_timer
 .text.vAnalogSensors_config_adc_dma
                0x000000000800158c      0x14c obj/mod_sensors.o
                0x000000000800158c                vAnalogSensors_config_adc_dma
 .text.vSensorsTask
                0x00000000080016d8      0x16c obj/mod_sensors.o
                0x00000000080016d8                vSensorsTask
 .text.vDrive_Configuration
                0x0000000008001844       0xfc obj/mod_drive.o
                0x0000000008001844                vDrive_Configuration
 .text.vDrive_motors_control
                0x0000000008001940       0xc0 obj/mod_drive.o
                0x0000000008001940                vDrive_motors_control
 .text.vDriveTask
                0x0000000008001a00       0x98 obj/mod_drive.o
                0x0000000008001a00                vDriveTask
 .text.vOrientation_sensor_configuration
                0x0000000008001a98      0x24c obj/mod_orientsensor.o
                0x0000000008001a98                vOrientation_sensor_configuration
 *fill*         0x0000000008001ce4        0x4 
 .text.gyro_get_data
                0x0000000008001ce8      0x264 obj/mod_orientsensor.o
                0x0000000008001ce8                gyro_get_data
 .text.sensor_get_yaw
                0x0000000008001f4c       0x14 obj/mod_orientsensor.o
                0x0000000008001f4c                sensor_get_yaw
 .text.sensor_get_pitch
                0x0000000008001f60       0x14 obj/mod_orientsensor.o
                0x0000000008001f60                sensor_get_pitch
 .text.sensor_get_roll
                0x0000000008001f74       0x14 obj/mod_orientsensor.o
                0x0000000008001f74                sensor_get_roll
 .text.sensor_update_acclx_tilt
                0x0000000008001f88       0x8c obj/mod_orientsensor.o
                0x0000000008001f88                sensor_update_acclx_tilt
 .text.sensor_get_tiltx
                0x0000000008002014        0xc obj/mod_orientsensor.o
                0x0000000008002014                sensor_get_tiltx
 .text.sensor_update_accly_tilt
                0x0000000008002020       0x9c obj/mod_orientsensor.o
                0x0000000008002020                sensor_update_accly_tilt
 .text.sensor_get_tilty
                0x00000000080020bc        0xc obj/mod_orientsensor.o
                0x00000000080020bc                sensor_get_tilty
 .text.sensor_update_acclz_tilt
                0x00000000080020c8       0xe4 obj/mod_orientsensor.o
                0x00000000080020c8                sensor_update_acclz_tilt
 *fill*         0x00000000080021ac        0x4 
 .text.accl_get_data
                0x00000000080021b0      0x1c4 obj/mod_orientsensor.o
                0x00000000080021b0                accl_get_data
 .text.sensor_get_tiltz
                0x0000000008002374        0xc obj/mod_orientsensor.o
                0x0000000008002374                sensor_get_tiltz
 .text.vLcdScrSaverCallback
                0x0000000008002380        0xc obj/mod_lcd.o
 .text.lcd_light_on_confirm
                0x000000000800238c       0x60 obj/mod_lcd.o
 .text.lcd_signal_on_confirm
                0x00000000080023ec       0x20 obj/mod_lcd.o
 .text.ZtScI2cMxWaitLcdReady.isra.0
                0x000000000800240c       0x48 obj/mod_lcd.o
 .text.ZtScI2cMxDisplay8x16Str.isra.0
                0x0000000008002454       0x74 obj/mod_lcd.o
 .text.ZtScI2cMxClearScreen.isra.0
                0x00000000080024c8       0x40 obj/mod_lcd.o
 .text.ZtScI2cMxDisplayArea
                0x0000000008002508       0x78 obj/mod_lcd.o
 .text.lcd_string_fill_and_display_degree
                0x0000000008002580       0x90 obj/mod_lcd.o
 .text.lcd_orient_display_state
                0x0000000008002610       0xe8 obj/mod_lcd.o
 .text.lcd_signal_display_state
                0x00000000080026f8       0x80 obj/mod_lcd.o
 .text.lcd_light_display_state
                0x0000000008002778       0x88 obj/mod_lcd.o
 .text.lcd_mainmenu_options.constprop.0
                0x0000000008002800       0xa8 obj/mod_lcd.o
 .text.lcd_status_display_options
                0x00000000080028a8       0xa8 obj/mod_lcd.o
 .text.lcd_light_display_options
                0x0000000008002950      0x124 obj/mod_lcd.o
 .text.lcd_mainmenu_scroll_title
                0x0000000008002a74      0x120 obj/mod_lcd.o
 .text.lcd_status_on_confirm
                0x0000000008002b94      0x1fc obj/mod_lcd.o
 .text.lcd_enviro_display_state
                0x0000000008002d90      0x200 obj/mod_lcd.o
 .text.lcd_status_display_state
                0x0000000008002f90      0x224 obj/mod_lcd.o
 .text.lcd_mov_display_state
                0x00000000080031b4      0x28c obj/mod_lcd.o
 .text.vLcdTask
                0x0000000008003440      0x87c obj/mod_lcd.o
                0x0000000008003440                vLcdTask
 .text.bLighting_is_auto
                0x0000000008003cbc       0x14 obj/mod_lighting.o
                0x0000000008003cbc                bLighting_is_auto
 .text.vLighting_set_auto
                0x0000000008003cd0        0xc obj/mod_lighting.o
                0x0000000008003cd0                vLighting_set_auto
 .text.vLighting_configuration
                0x0000000008003cdc       0x68 obj/mod_lighting.o
                0x0000000008003cdc                vLighting_configuration
 .text.vLighting_set_state
                0x0000000008003d44       0x68 obj/mod_lighting.o
                0x0000000008003d44                vLighting_set_state
 .text.vGet_Lighting_State
                0x0000000008003dac       0x4c obj/mod_lighting.o
                0x0000000008003dac                vGet_Lighting_State
 .text.vLighting_get_state
                0x0000000008003df8       0x20 obj/mod_lighting.o
                0x0000000008003df8                vLighting_get_state
 .text.vLighting_adjust_auto_light
                0x0000000008003e18      0x158 obj/mod_lighting.o
                0x0000000008003e18                vLighting_adjust_auto_light
 .text.vSound_setup
                0x0000000008003f70       0x28 obj/mod_console.o
 .text.vLight_mode
                0x0000000008003f98       0x3c obj/mod_console.o
 .text.vDrive_speed
                0x0000000008003fd4       0x4c obj/mod_console.o
 .text.vDrive_setup
                0x0000000008004020       0x20 obj/mod_console.o
 .text.console_print_menu_header
                0x0000000008004040       0x68 obj/mod_console.o
 .text.console_print_sub_menu
                0x00000000080040a8       0x7c obj/mod_console.o
 .text.vLight_setup
                0x0000000008004124       0x28 obj/mod_console.o
 .text.console_print_main_menu.constprop.0
                0x000000000800414c       0x60 obj/mod_console.o
 .text.vSound_mode
                0x00000000080041ac       0x4c obj/mod_console.o
 .text.vConsoleTask
                0x00000000080041f8      0x178 obj/mod_console.o
                0x00000000080041f8                vConsoleTask
 .text.vUSB_configuration
                0x0000000008004370       0x14 obj/mod_console.o
                0x0000000008004370                vUSB_configuration
 .text.__io_write
                0x0000000008004384       0x38 obj/mod_console.o
                0x0000000008004384                __io_write
 .text.__io_read
                0x00000000080043bc       0x7c obj/mod_console.o
                0x00000000080043bc                __io_read
 .text.vI2C_restart_transmission
                0x0000000008004438       0x40 obj/mod_i2c.o
 .text.vI2C_stop_transmission
                0x0000000008004478       0x50 obj/mod_i2c.o
 .text.xI2C_write_byte
                0x00000000080044c8       0xa0 obj/mod_i2c.o
 .text.xI2C_read_byte
                0x0000000008004568       0x9c obj/mod_i2c.o
 .text.xI2C_read_byte_slow
                0x0000000008004604       0x9c obj/mod_i2c.o
 .text.xI2C_write_sequence
                0x00000000080046a0       0x64 obj/mod_i2c.o
                0x00000000080046a0                xI2C_write_sequence
 .text.xI2C_read_sequence
                0x0000000008004704       0x80 obj/mod_i2c.o
                0x0000000008004704                xI2C_read_sequence
 .text.xI2C_read_sequence_slow
                0x0000000008004784       0x80 obj/mod_i2c.o
                0x0000000008004784                xI2C_read_sequence_slow
 .text.vI2C_configuration
                0x0000000008004804       0x34 obj/mod_i2c.o
                0x0000000008004804                vI2C_configuration
 .text.radio_recv_handler
                0x0000000008004838       0x84 obj/mod_radio.o
 .text.radio_send_command
                0x00000000080048bc       0x9c obj/mod_radio.o
 .text.radio_get_config
                0x0000000008004958       0xa0 obj/mod_radio.o
 .text.radio_get_payload
                0x00000000080049f8       0x34 obj/mod_radio.o
 .text.radio_get_rfdata_rate
                0x0000000008004a2c       0x34 obj/mod_radio.o
 .text.radio_get_uart_baudrate
                0x0000000008004a60       0x34 obj/mod_radio.o
 .text.radio_get_txrx_led
                0x0000000008004a94       0x8c obj/mod_radio.o
 .text.radio_get_modulation
                0x0000000008004b20       0x8c obj/mod_radio.o
 .text.radio_get_freq
                0x0000000008004bac       0x68 obj/mod_radio.o
 .text.print_results
                0x0000000008004c14       0x40 obj/mod_radio.o
                0x0000000008004c14                print_results
 .text.vRadio_configuration
                0x0000000008004c54       0x98 obj/mod_radio.o
                0x0000000008004c54                vRadio_configuration
 .text.radio_recv_handler_irq
                0x0000000008004cec        0x8 obj/mod_radio.o
                0x0000000008004cec                radio_recv_handler_irq
 .text.vRadioTask
                0x0000000008004cf4      0x19c obj/mod_radio.o
                0x0000000008004cf4                vRadioTask
 .text.auxil_get_bit_from_pos
                0x0000000008004e90       0x14 obj/mod_auxiliary.o
                0x0000000008004e90                auxil_get_bit_from_pos
 .text.radio_get_status_curr
                0x0000000008004ea4       0x74 obj/mod_auxiliary.o
                0x0000000008004ea4                radio_get_status_curr
 .text.radio_set_status_curr
                0x0000000008004f18       0x6c obj/mod_auxiliary.o
                0x0000000008004f18                radio_set_status_curr
 .text.exec_group_set_status
                0x0000000008004f84       0x5c obj/mod_auxiliary.o
                0x0000000008004f84                exec_group_set_status
 .text.exec_group_get_status
                0x0000000008004fe0       0x54 obj/mod_auxiliary.o
                0x0000000008004fe0                exec_group_get_status
 .text.drive_get_status
                0x0000000008005034       0x5c obj/mod_auxiliary.o
                0x0000000008005034                drive_get_status
 .text.drive_get_direction
                0x0000000008005090       0x44 obj/mod_auxiliary.o
                0x0000000008005090                drive_get_direction
 .text.drive_set_status
                0x00000000080050d4       0x5c obj/mod_auxiliary.o
                0x00000000080050d4                drive_set_status
 .text.light_get_leds_on
                0x0000000008005130       0x48 obj/mod_auxiliary.o
                0x0000000008005130                light_get_leds_on
 .text.light_is_auto
                0x0000000008005178       0x4c obj/mod_auxiliary.o
                0x0000000008005178                light_is_auto
 .text.light_set_auto
                0x00000000080051c4       0x5c obj/mod_auxiliary.o
                0x00000000080051c4                light_set_auto
 .text.light_toggle_led
                0x0000000008005220       0x68 obj/mod_auxiliary.o
                0x0000000008005220                light_toggle_led
 .text.light_set_led_on
                0x0000000008005288       0x6c obj/mod_auxiliary.o
                0x0000000008005288                light_set_led_on
 .text.light_off_all
                0x00000000080052f4       0x58 obj/mod_auxiliary.o
                0x00000000080052f4                light_off_all
 .text.signal_is_on
                0x000000000800534c       0x4c obj/mod_auxiliary.o
                0x000000000800534c                signal_is_on
 .text.signal_set_state
                0x0000000008005398       0x5c obj/mod_auxiliary.o
                0x0000000008005398                signal_set_state
 .text.SystemInit
                0x00000000080053f4      0x100 /tmp/ccPh1D7g.o
                0x00000000080053f4                SystemInit
 .text.NMI_Handler
                0x00000000080054f4        0x4 /tmp/cchESTC9.o
                0x00000000080054f4                NMI_Handler
 .text.HardFault_Handler
                0x00000000080054f8        0x4 /tmp/cchESTC9.o
                0x00000000080054f8                HardFault_Handler
 .text.MemManage_Handler
                0x00000000080054fc        0x4 /tmp/cchESTC9.o
                0x00000000080054fc                MemManage_Handler
 .text.BusFault_Handler
                0x0000000008005500        0x4 /tmp/cchESTC9.o
                0x0000000008005500                BusFault_Handler
 .text.UsageFault_Handler
                0x0000000008005504        0x4 /tmp/cchESTC9.o
                0x0000000008005504                UsageFault_Handler
 .text.DebugMon_Handler
                0x0000000008005508        0x4 /tmp/cchESTC9.o
                0x0000000008005508                DebugMon_Handler
 .text.USB_LP_CAN1_RX0_IRQHandler
                0x000000000800550c        0x4 /tmp/cchESTC9.o
                0x000000000800550c                USB_LP_CAN1_RX0_IRQHandler
 .text.USART1_IRQHandler
                0x0000000008005510       0x2c /tmp/cchESTC9.o
                0x0000000008005510                USART1_IRQHandler
 .text.TIM1_CC_IRQHandler
                0x000000000800553c       0x24 /tmp/cchESTC9.o
                0x000000000800553c                TIM1_CC_IRQHandler
 .text.TIM2_IRQHandler
                0x0000000008005560        0x4 /tmp/cchESTC9.o
                0x0000000008005560                TIM2_IRQHandler
 .text.TIM3_IRQHandler
                0x0000000008005564        0x4 /tmp/cchESTC9.o
                0x0000000008005564                TIM3_IRQHandler
 .text.TIM4_IRQHandler
                0x0000000008005568       0xa0 /tmp/cchESTC9.o
                0x0000000008005568                TIM4_IRQHandler
 .text.PVD_IRQHandler
                0x0000000008005608       0x28 /tmp/cchESTC9.o
                0x0000000008005608                PVD_IRQHandler
 .text.__set_MSP
                0x0000000008005630        0x8 /tmp/cc9ErQe2.o
                0x0000000008005630                __set_MSP
 .text.startup.main
                0x0000000008005638      0x178 /tmp/cc9a8G1U.o
                0x0000000008005638                main
 .text.vApplicationStackOverflowHook
                0x00000000080057b0       0x10 /tmp/cc9a8G1U.o
                0x00000000080057b0                vApplicationStackOverflowHook
 .text._getpid  0x00000000080057c0        0x4 /tmp/ccTyJAXN.o
                0x00000000080057c0                _getpid
 .text._kill    0x00000000080057c4       0x14 /tmp/ccTyJAXN.o
                0x00000000080057c4                _kill
 .text._exit    0x00000000080057d8        0xc /tmp/ccTyJAXN.o
                0x00000000080057d8                _exit
 .text._read    0x00000000080057e4        0x8 /tmp/ccTyJAXN.o
                0x00000000080057e4                _read
 .text._write   0x00000000080057ec        0x8 /tmp/ccTyJAXN.o
                0x00000000080057ec                _write
 .text._close   0x00000000080057f4        0x8 /tmp/ccTyJAXN.o
                0x00000000080057f4                _close
 .text._fstat   0x00000000080057fc        0xc /tmp/ccTyJAXN.o
                0x00000000080057fc                _fstat
 .text._isatty  0x0000000008005808        0x4 /tmp/ccTyJAXN.o
                0x0000000008005808                _isatty
 .text._lseek   0x000000000800580c        0x4 /tmp/ccTyJAXN.o
                0x000000000800580c                _lseek
 .text._malloc_r
                0x0000000008005810        0x8 /tmp/ccrbnG3G.o
                0x0000000008005810                _malloc_r
 .text._calloc_r
                0x0000000008005818        0x8 /tmp/ccrbnG3G.o
                0x0000000008005818                _calloc_r
 .text._realloc_r
                0x0000000008005820       0x2c /tmp/ccrbnG3G.o
                0x0000000008005820                _realloc_r
 .text._free_r  0x000000000800584c        0x8 /tmp/ccrbnG3G.o
                0x000000000800584c                _free_r
 .text.Reset_Handler
                0x0000000008005854       0x44 /tmp/ccJdFGaA.o
                0x0000000008005854                Reset_Handler
 .text.Default_Handler
                0x0000000008005898        0x2 /tmp/ccJdFGaA.o
                0x0000000008005898                EXTI2_IRQHandler
                0x0000000008005898                EXTI3_IRQHandler
                0x0000000008005898                EXTI0_IRQHandler
                0x0000000008005898                I2C2_EV_IRQHandler
                0x0000000008005898                ADC1_2_IRQHandler
                0x0000000008005898                SPI1_IRQHandler
                0x0000000008005898                TAMPER_IRQHandler
                0x0000000008005898                DMA1_Channel4_IRQHandler
                0x0000000008005898                USART3_IRQHandler
                0x0000000008005898                RTC_IRQHandler
                0x0000000008005898                DMA1_Channel7_IRQHandler
                0x0000000008005898                CAN1_RX1_IRQHandler
                0x0000000008005898                I2C1_EV_IRQHandler
                0x0000000008005898                DMA1_Channel6_IRQHandler
                0x0000000008005898                RCC_IRQHandler
                0x0000000008005898                TIM1_TRG_COM_IRQHandler
                0x0000000008005898                DMA1_Channel1_IRQHandler
                0x0000000008005898                Default_Handler
                0x0000000008005898                EXTI15_10_IRQHandler
                0x0000000008005898                EXTI9_5_IRQHandler
                0x0000000008005898                SPI2_IRQHandler
                0x0000000008005898                DMA1_Channel5_IRQHandler
                0x0000000008005898                EXTI4_IRQHandler
                0x0000000008005898                USB_HP_CAN1_TX_IRQHandler
                0x0000000008005898                DMA1_Channel3_IRQHandler
                0x0000000008005898                TIM1_UP_IRQHandler
                0x0000000008005898                WWDG_IRQHandler
                0x0000000008005898                TIM1_BRK_IRQHandler
                0x0000000008005898                EXTI1_IRQHandler
                0x0000000008005898                RTCAlarm_IRQHandler
                0x0000000008005898                USART2_IRQHandler
                0x0000000008005898                I2C2_ER_IRQHandler
                0x0000000008005898                DMA1_Channel2_IRQHandler
                0x0000000008005898                CAN1_SCE_IRQHandler
                0x0000000008005898                FLASH_IRQHandler
                0x0000000008005898                I2C1_ER_IRQHandler
                0x0000000008005898                USBWakeUp_IRQHandler
 *fill*         0x000000000800589a        0x2 
 .text.Set_System
                0x000000000800589c       0x38 lib/libusb.a(hw_config.o)
                0x000000000800589c                Set_System
 .text.Set_USBClock
                0x00000000080058d4       0x18 lib/libusb.a(hw_config.o)
                0x00000000080058d4                Set_USBClock
 .text.USB_Cable_Config
                0x00000000080058ec       0x18 lib/libusb.a(hw_config.o)
                0x00000000080058ec                USB_Cable_Config
 .text.USB_Receive_Data
                0x0000000008005904        0xc lib/libusb.a(hw_config.o)
                0x0000000008005904                USB_Receive_Data
 .text.Handle_USBAsynchXfer
                0x0000000008005910       0x88 lib/libusb.a(hw_config.o)
                0x0000000008005910                Handle_USBAsynchXfer
 .text.Get_SerialNum
                0x0000000008005998       0x84 lib/libusb.a(hw_config.o)
                0x0000000008005998                Get_SerialNum
 .text.EP1_IN_Callback
                0x0000000008005a1c       0x5c lib/libusb.a(usb_endp.o)
                0x0000000008005a1c                EP1_IN_Callback
 .text.EP3_OUT_Callback
                0x0000000008005a78       0x30 lib/libusb.a(usb_endp.o)
                0x0000000008005a78                EP3_OUT_Callback
 .text.SOF_Callback
                0x0000000008005aa8       0x2c lib/libusb.a(usb_endp.o)
                0x0000000008005aa8                SOF_Callback
 .text.USB_Istr
                0x0000000008005ad4       0x78 lib/libusb.a(usb_istr.o)
                0x0000000008005ad4                USB_Istr
 .text.Virtual_Com_Port_SetConfiguration
                0x0000000008005b4c       0x18 lib/libusb.a(usb_prop.o)
                0x0000000008005b4c                Virtual_Com_Port_SetConfiguration
 .text.Virtual_Com_Port_SetDeviceAddress
                0x0000000008005b64        0xc lib/libusb.a(usb_prop.o)
                0x0000000008005b64                Virtual_Com_Port_SetDeviceAddress
 .text.Virtual_Com_Port_Status_In
                0x0000000008005b70       0x14 lib/libusb.a(usb_prop.o)
                0x0000000008005b70                Virtual_Com_Port_Status_In
 .text.Virtual_Com_Port_Status_Out
                0x0000000008005b84        0x4 lib/libusb.a(usb_prop.o)
                0x0000000008005b84                Virtual_Com_Port_Status_Out
 .text.Virtual_Com_Port_Data_Setup
                0x0000000008005b88       0x5c lib/libusb.a(usb_prop.o)
                0x0000000008005b88                Virtual_Com_Port_Data_Setup
 .text.Virtual_Com_Port_Get_Interface_Setting
                0x0000000008005be4       0x10 lib/libusb.a(usb_prop.o)
                0x0000000008005be4                Virtual_Com_Port_Get_Interface_Setting
 .text.Virtual_Com_Port_GetLineCoding
                0x0000000008005bf4       0x18 lib/libusb.a(usb_prop.o)
                0x0000000008005bf4                Virtual_Com_Port_GetLineCoding
 .text.Virtual_Com_Port_init
                0x0000000008005c0c       0x24 lib/libusb.a(usb_prop.o)
                0x0000000008005c0c                Virtual_Com_Port_init
 .text.Virtual_Com_Port_Reset
                0x0000000008005c30       0xe0 lib/libusb.a(usb_prop.o)
                0x0000000008005c30                Virtual_Com_Port_Reset
 .text.Virtual_Com_Port_GetDeviceDescriptor
                0x0000000008005d10        0xc lib/libusb.a(usb_prop.o)
                0x0000000008005d10                Virtual_Com_Port_GetDeviceDescriptor
 .text.Virtual_Com_Port_GetConfigDescriptor
                0x0000000008005d1c        0xc lib/libusb.a(usb_prop.o)
                0x0000000008005d1c                Virtual_Com_Port_GetConfigDescriptor
 .text.Virtual_Com_Port_GetStringDescriptor
                0x0000000008005d28       0x20 lib/libusb.a(usb_prop.o)
                0x0000000008005d28                Virtual_Com_Port_GetStringDescriptor
 .text.Virtual_Com_Port_NoData_Setup
                0x0000000008005d48       0x24 lib/libusb.a(usb_prop.o)
                0x0000000008005d48                Virtual_Com_Port_NoData_Setup
 .text.Virtual_Com_Port_SetLineCoding
                0x0000000008005d6c       0x18 lib/libusb.a(usb_prop.o)
                0x0000000008005d6c                Virtual_Com_Port_SetLineCoding
 .text.PowerOn  0x0000000008005d84       0x30 lib/libusb.a(usb_pwr.o)
                0x0000000008005d84                PowerOn
 .text.Standard_GetConfiguration
                0x0000000008005db4       0x28 lib/libusb.a(usb_core.o)
                0x0000000008005db4                Standard_GetConfiguration
 .text.Standard_GetInterface
                0x0000000008005ddc       0x28 lib/libusb.a(usb_core.o)
                0x0000000008005ddc                Standard_GetInterface
 .text.Standard_GetStatus
                0x0000000008005e04       0x90 lib/libusb.a(usb_core.o)
                0x0000000008005e04                Standard_GetStatus
 .text.DataStageIn
                0x0000000008005e94       0xb4 lib/libusb.a(usb_core.o)
 .text.Standard_SetConfiguration
                0x0000000008005f48       0x38 lib/libusb.a(usb_core.o)
                0x0000000008005f48                Standard_SetConfiguration
 .text.Standard_SetInterface
                0x0000000008005f80       0x4c lib/libusb.a(usb_core.o)
                0x0000000008005f80                Standard_SetInterface
 .text.Standard_ClearFeature
                0x0000000008005fcc       0xf0 lib/libusb.a(usb_core.o)
                0x0000000008005fcc                Standard_ClearFeature
 .text.Standard_SetEndPointFeature
                0x00000000080060bc       0x9c lib/libusb.a(usb_core.o)
                0x00000000080060bc                Standard_SetEndPointFeature
 .text.Standard_GetDescriptorData
                0x0000000008006158       0x1c lib/libusb.a(usb_core.o)
                0x0000000008006158                Standard_GetDescriptorData
 .text.Setup0_Process
                0x0000000008006174      0x39c lib/libusb.a(usb_core.o)
                0x0000000008006174                Setup0_Process
 .text.In0_Process
                0x0000000008006510       0xcc lib/libusb.a(usb_core.o)
                0x0000000008006510                In0_Process
 .text.Out0_Process
                0x00000000080065dc      0x130 lib/libusb.a(usb_core.o)
                0x00000000080065dc                Out0_Process
 .text.SetDeviceAddress
                0x000000000800670c       0x44 lib/libusb.a(usb_core.o)
                0x000000000800670c                SetDeviceAddress
 .text.NOP_Process
                0x0000000008006750        0x4 lib/libusb.a(usb_core.o)
                0x0000000008006750                NOP_Process
 .text.USB_Init
                0x0000000008006754       0x38 lib/libusb.a(usb_init.o)
                0x0000000008006754                USB_Init
 .text.CTR_LP   0x000000000800678c      0x1a8 lib/libusb.a(usb_int.o)
                0x000000000800678c                CTR_LP
 .text.UserToPMABufferCopy
                0x0000000008006934       0x20 lib/libusb.a(usb_mem.o)
                0x0000000008006934                UserToPMABufferCopy
 .text.PMAToUserBufferCopy
                0x0000000008006954       0x24 lib/libusb.a(usb_mem.o)
                0x0000000008006954                PMAToUserBufferCopy
 .text.SetBTABLE
                0x0000000008006978       0x14 lib/libusb.a(usb_regs.o)
                0x0000000008006978                SetBTABLE
 .text.SetEPType
                0x000000000800698c       0x1c lib/libusb.a(usb_regs.o)
                0x000000000800698c                SetEPType
 .text.SetEPTxStatus
                0x00000000080069a8       0x34 lib/libusb.a(usb_regs.o)
                0x00000000080069a8                SetEPTxStatus
 .text.SetEPRxStatus
                0x00000000080069dc       0x34 lib/libusb.a(usb_regs.o)
                0x00000000080069dc                SetEPRxStatus
 .text.SetEPTxValid
                0x0000000008006a10       0x28 lib/libusb.a(usb_regs.o)
                0x0000000008006a10                SetEPTxValid
 .text.SetEPRxValid
                0x0000000008006a38       0x28 lib/libusb.a(usb_regs.o)
                0x0000000008006a38                SetEPRxValid
 .text.Clear_Status_Out
                0x0000000008006a60       0x20 lib/libusb.a(usb_regs.o)
                0x0000000008006a60                Clear_Status_Out
 .text.ClearDTOG_RX
                0x0000000008006a80       0x24 lib/libusb.a(usb_regs.o)
                0x0000000008006a80                ClearDTOG_RX
 .text.ClearDTOG_TX
                0x0000000008006aa4       0x24 lib/libusb.a(usb_regs.o)
                0x0000000008006aa4                ClearDTOG_TX
 .text.SetEPTxAddr
                0x0000000008006ac8       0x24 lib/libusb.a(usb_regs.o)
                0x0000000008006ac8                SetEPTxAddr
 .text.SetEPRxAddr
                0x0000000008006aec       0x24 lib/libusb.a(usb_regs.o)
                0x0000000008006aec                SetEPRxAddr
 .text.GetEPTxAddr
                0x0000000008006b10       0x20 lib/libusb.a(usb_regs.o)
                0x0000000008006b10                GetEPTxAddr
 .text.GetEPRxAddr
                0x0000000008006b30       0x20 lib/libusb.a(usb_regs.o)
                0x0000000008006b30                GetEPRxAddr
 .text.SetEPTxCount
                0x0000000008006b50       0x20 lib/libusb.a(usb_regs.o)
                0x0000000008006b50                SetEPTxCount
 .text.SetEPRxCount
                0x0000000008006b70       0x44 lib/libusb.a(usb_regs.o)
                0x0000000008006b70                SetEPRxCount
 .text.GetEPRxCount
                0x0000000008006bb4       0x24 lib/libusb.a(usb_regs.o)
                0x0000000008006bb4                GetEPRxCount
 .text.ByteSwap
                0x0000000008006bd8        0x8 lib/libusb.a(usb_regs.o)
                0x0000000008006bd8                ByteSwap
 .text.USB_SIL_Init
                0x0000000008006be0       0x20 lib/libusb.a(usb_sil.o)
                0x0000000008006be0                USB_SIL_Init
 .text.USB_SIL_Read
                0x0000000008006c00       0x24 lib/libusb.a(usb_sil.o)
                0x0000000008006c00                USB_SIL_Read
 .text.prvSearchForNameWithinSingleList
                0x0000000008006c24       0x58 lib/libfreertos.a(tasks.o)
 .text.prvAddCurrentTaskToDelayedList
                0x0000000008006c7c       0x8c lib/libfreertos.a(tasks.o)
 .text.prvIdleTask
                0x0000000008006d08       0x74 lib/libfreertos.a(tasks.o)
 .text.xTaskCreate
                0x0000000008006d7c      0x1f8 lib/libfreertos.a(tasks.o)
                0x0000000008006d7c                xTaskCreate
 .text.uxTaskPriorityGet
                0x0000000008006f74       0x28 lib/libfreertos.a(tasks.o)
                0x0000000008006f74                uxTaskPriorityGet
 .text.vTaskPrioritySet
                0x0000000008006f9c       0xd4 lib/libfreertos.a(tasks.o)
                0x0000000008006f9c                vTaskPrioritySet
 .text.vTaskStartScheduler
                0x0000000008007070       0x80 lib/libfreertos.a(tasks.o)
                0x0000000008007070                vTaskStartScheduler
 .text.vTaskSuspendAll
                0x00000000080070f0       0x10 lib/libfreertos.a(tasks.o)
                0x00000000080070f0                vTaskSuspendAll
 .text.xTaskGetTickCount
                0x0000000008007100        0xc lib/libfreertos.a(tasks.o)
                0x0000000008007100                xTaskGetTickCount
 .text.xTaskIncrementTick
                0x000000000800710c      0x154 lib/libfreertos.a(tasks.o)
                0x000000000800710c                xTaskIncrementTick
 .text.xTaskResumeAll
                0x0000000008007260      0x118 lib/libfreertos.a(tasks.o)
                0x0000000008007260                xTaskResumeAll
 .text.vTaskDelayUntil
                0x0000000008007378       0x54 lib/libfreertos.a(tasks.o)
                0x0000000008007378                vTaskDelayUntil
 .text.vTaskDelay
                0x00000000080073cc       0x38 lib/libfreertos.a(tasks.o)
                0x00000000080073cc                vTaskDelay
 .text.xTaskGetHandle
                0x0000000008007404       0x9c lib/libfreertos.a(tasks.o)
                0x0000000008007404                xTaskGetHandle
 .text.vTaskSwitchContext
                0x00000000080074a0       0x80 lib/libfreertos.a(tasks.o)
                0x00000000080074a0                vTaskSwitchContext
 .text.vTaskPlaceOnEventList
                0x0000000008007520       0x20 lib/libfreertos.a(tasks.o)
                0x0000000008007520                vTaskPlaceOnEventList
 .text.vTaskPlaceOnEventListRestricted
                0x0000000008007540       0x2c lib/libfreertos.a(tasks.o)
                0x0000000008007540                vTaskPlaceOnEventListRestricted
 .text.xTaskRemoveFromEventList
                0x000000000800756c       0x78 lib/libfreertos.a(tasks.o)
                0x000000000800756c                xTaskRemoveFromEventList
 .text.vTaskInternalSetTimeOutState
                0x00000000080075e4       0x18 lib/libfreertos.a(tasks.o)
                0x00000000080075e4                vTaskInternalSetTimeOutState
 .text.xTaskCheckForTimeOut
                0x00000000080075fc       0x6c lib/libfreertos.a(tasks.o)
                0x00000000080075fc                xTaskCheckForTimeOut
 .text.vTaskMissedYield
                0x0000000008007668        0xc lib/libfreertos.a(tasks.o)
                0x0000000008007668                vTaskMissedYield
 .text.xTaskGetSchedulerState
                0x0000000008007674       0x20 lib/libfreertos.a(tasks.o)
                0x0000000008007674                xTaskGetSchedulerState
 .text.xTaskPriorityInherit
                0x0000000008007694       0xa8 lib/libfreertos.a(tasks.o)
                0x0000000008007694                xTaskPriorityInherit
 .text.xTaskPriorityDisinherit
                0x000000000800773c       0x7c lib/libfreertos.a(tasks.o)
                0x000000000800773c                xTaskPriorityDisinherit
 .text.vTaskPriorityDisinheritAfterTimeout
                0x00000000080077b8       0x9c lib/libfreertos.a(tasks.o)
                0x00000000080077b8                vTaskPriorityDisinheritAfterTimeout
 .text.pvTaskIncrementMutexHeldCount
                0x0000000008007854       0x18 lib/libfreertos.a(tasks.o)
                0x0000000008007854                pvTaskIncrementMutexHeldCount
 .text.xTaskNotifyWait
                0x000000000800786c       0x9c lib/libfreertos.a(tasks.o)
                0x000000000800786c                xTaskNotifyWait
 .text.xTaskGenericNotifyFromISR
                0x0000000008007908       0xe4 lib/libfreertos.a(tasks.o)
                0x0000000008007908                xTaskGenericNotifyFromISR
 .text.vListInitialise
                0x00000000080079ec       0x18 lib/libfreertos.a(list.o)
                0x00000000080079ec                vListInitialise
 .text.vListInitialiseItem
                0x0000000008007a04        0x8 lib/libfreertos.a(list.o)
                0x0000000008007a04                vListInitialiseItem
 .text.vListInsertEnd
                0x0000000008007a0c       0x1c lib/libfreertos.a(list.o)
                0x0000000008007a0c                vListInsertEnd
 .text.vListInsert
                0x0000000008007a28       0x30 lib/libfreertos.a(list.o)
                0x0000000008007a28                vListInsert
 .text.uxListRemove
                0x0000000008007a58       0x24 lib/libfreertos.a(list.o)
                0x0000000008007a58                uxListRemove
 .text.prvCopyDataToQueue
                0x0000000008007a7c       0x70 lib/libfreertos.a(queue.o)
 .text.prvUnlockQueue
                0x0000000008007aec       0x8c lib/libfreertos.a(queue.o)
 .text.xQueueGenericCreate
                0x0000000008007b78       0x7c lib/libfreertos.a(queue.o)
                0x0000000008007b78                xQueueGenericCreate
 .text.xQueueGenericSend
                0x0000000008007bf4      0x114 lib/libfreertos.a(queue.o)
                0x0000000008007bf4                xQueueGenericSend
 .text.xQueueCreateMutex
                0x0000000008007d08       0x6c lib/libfreertos.a(queue.o)
                0x0000000008007d08                xQueueCreateMutex
 .text.xQueueGenericSendFromISR
                0x0000000008007d74       0x74 lib/libfreertos.a(queue.o)
                0x0000000008007d74                xQueueGenericSendFromISR
 .text.xQueueReceive
                0x0000000008007de8      0x128 lib/libfreertos.a(queue.o)
                0x0000000008007de8                xQueueReceive
 .text.xQueueSemaphoreTake
                0x0000000008007f10      0x174 lib/libfreertos.a(queue.o)
                0x0000000008007f10                xQueueSemaphoreTake
 .text.vQueueWaitForMessageRestricted
                0x0000000008008084       0x50 lib/libfreertos.a(queue.o)
                0x0000000008008084                vQueueWaitForMessageRestricted
 .text.prvCheckForValidListAndQueue
                0x00000000080080d4       0x54 lib/libfreertos.a(timers.o)
 .text.xTimerCreateTimerTask
                0x0000000008008128       0x38 lib/libfreertos.a(timers.o)
                0x0000000008008128                xTimerCreateTimerTask
 .text.xTimerCreate
                0x0000000008008160       0x34 lib/libfreertos.a(timers.o)
                0x0000000008008160                xTimerCreate
 .text.xTimerGenericCommand
                0x0000000008008194       0x58 lib/libfreertos.a(timers.o)
                0x0000000008008194                xTimerGenericCommand
 .text.prvSwitchTimerLists
                0x00000000080081ec       0x70 lib/libfreertos.a(timers.o)
 .text.prvTimerTask
                0x000000000800825c      0x204 lib/libfreertos.a(timers.o)
 .text.prvTaskExitError
                0x0000000008008460       0x20 lib/libfreertos.a(port.o)
 .text.prvPortStartFirstTask
                0x0000000008008480       0x20 lib/libfreertos.a(port.o)
 .text.pxPortInitialiseStack
                0x00000000080084a0       0x24 lib/libfreertos.a(port.o)
                0x00000000080084a0                pxPortInitialiseStack
 *fill*         0x00000000080084c4        0xc 
 .text.SVC_Handler
                0x00000000080084d0       0x24 lib/libfreertos.a(port.o)
                0x00000000080084d0                SVC_Handler
 .text.vPortEnterCritical
                0x00000000080084f4       0x20 lib/libfreertos.a(port.o)
                0x00000000080084f4                vPortEnterCritical
 .text.vPortExitCritical
                0x0000000008008514       0x14 lib/libfreertos.a(port.o)
                0x0000000008008514                vPortExitCritical
 *fill*         0x0000000008008528        0x8 
 .text.PendSV_Handler
                0x0000000008008530       0x44 lib/libfreertos.a(port.o)
                0x0000000008008530                PendSV_Handler
 .text.SysTick_Handler
                0x0000000008008574       0x2c lib/libfreertos.a(port.o)
                0x0000000008008574                SysTick_Handler
 .text.vPortSetupTimerInterrupt
                0x00000000080085a0       0x18 lib/libfreertos.a(port.o)
                0x00000000080085a0                vPortSetupTimerInterrupt
 .text.xPortStartScheduler
                0x00000000080085b8       0x54 lib/libfreertos.a(port.o)
                0x00000000080085b8                xPortStartScheduler
 .text.prvInsertBlockIntoFreeList
                0x000000000800860c       0x60 lib/libfreertos.a(heap_4.o)
 .text.pvPortMalloc
                0x000000000800866c      0x100 lib/libfreertos.a(heap_4.o)
                0x000000000800866c                pvPortMalloc
 .text.vPortFree
                0x000000000800876c       0x50 lib/libfreertos.a(heap_4.o)
                0x000000000800876c                vPortFree
 .text.NVIC_PriorityGroupConfig
                0x00000000080087bc       0x14 lib/libstm32.a(misc.o)
                0x00000000080087bc                NVIC_PriorityGroupConfig
 .text.NVIC_Init
                0x00000000080087d0       0x6c lib/libstm32.a(misc.o)
                0x00000000080087d0                NVIC_Init
 .text.RCC_DeInit
                0x000000000800883c       0x40 lib/libstm32.a(stm32f10x_rcc.o)
                0x000000000800883c                RCC_DeInit
 .text.RCC_USBCLKConfig
                0x000000000800887c        0xc lib/libstm32.a(stm32f10x_rcc.o)
                0x000000000800887c                RCC_USBCLKConfig
 .text.RCC_GetClocksFreq
                0x0000000008008888       0x90 lib/libstm32.a(stm32f10x_rcc.o)
                0x0000000008008888                RCC_GetClocksFreq
 .text.RCC_AHBPeriphClockCmd
                0x0000000008008918       0x18 lib/libstm32.a(stm32f10x_rcc.o)
                0x0000000008008918                RCC_AHBPeriphClockCmd
 .text.RCC_APB2PeriphClockCmd
                0x0000000008008930       0x18 lib/libstm32.a(stm32f10x_rcc.o)
                0x0000000008008930                RCC_APB2PeriphClockCmd
 .text.RCC_APB1PeriphClockCmd
                0x0000000008008948       0x18 lib/libstm32.a(stm32f10x_rcc.o)
                0x0000000008008948                RCC_APB1PeriphClockCmd
 .text.RCC_APB2PeriphResetCmd
                0x0000000008008960       0x18 lib/libstm32.a(stm32f10x_rcc.o)
                0x0000000008008960                RCC_APB2PeriphResetCmd
 .text.RCC_GetFlagStatus
                0x0000000008008978       0x34 lib/libstm32.a(stm32f10x_rcc.o)
                0x0000000008008978                RCC_GetFlagStatus
 .text.GPIO_Init
                0x00000000080089ac       0xc4 lib/libstm32.a(stm32f10x_gpio.o)
                0x00000000080089ac                GPIO_Init
 .text.GPIO_ReadInputDataBit
                0x0000000008008a70        0xc lib/libstm32.a(stm32f10x_gpio.o)
                0x0000000008008a70                GPIO_ReadInputDataBit
 .text.GPIO_ReadOutputDataBit
                0x0000000008008a7c        0xc lib/libstm32.a(stm32f10x_gpio.o)
                0x0000000008008a7c                GPIO_ReadOutputDataBit
 .text.GPIO_SetBits
                0x0000000008008a88        0x4 lib/libstm32.a(stm32f10x_gpio.o)
                0x0000000008008a88                GPIO_SetBits
 .text.GPIO_ResetBits
                0x0000000008008a8c        0x4 lib/libstm32.a(stm32f10x_gpio.o)
                0x0000000008008a8c                GPIO_ResetBits
 .text.GPIO_WriteBit
                0x0000000008008a90        0xc lib/libstm32.a(stm32f10x_gpio.o)
                0x0000000008008a90                GPIO_WriteBit
 .text.GPIO_PinRemapConfig
                0x0000000008008a9c       0x70 lib/libstm32.a(stm32f10x_gpio.o)
                0x0000000008008a9c                GPIO_PinRemapConfig
 .text.EXTI_Init
                0x0000000008008b0c       0x80 lib/libstm32.a(stm32f10x_exti.o)
                0x0000000008008b0c                EXTI_Init
 .text.EXTI_GetITStatus
                0x0000000008008b8c       0x1c lib/libstm32.a(stm32f10x_exti.o)
                0x0000000008008b8c                EXTI_GetITStatus
 .text.EXTI_ClearITPendingBit
                0x0000000008008ba8        0xc lib/libstm32.a(stm32f10x_exti.o)
                0x0000000008008ba8                EXTI_ClearITPendingBit
 .text.TI1_Config
                0x0000000008008bb4       0x90 lib/libstm32.a(stm32f10x_tim.o)
 .text.TI2_Config
                0x0000000008008c44       0x88 lib/libstm32.a(stm32f10x_tim.o)
 .text.TIM_TimeBaseInit
                0x0000000008008ccc       0xac lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008008ccc                TIM_TimeBaseInit
 .text.TIM_OC1Init
                0x0000000008008d78       0x9c lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008008d78                TIM_OC1Init
 .text.TIM_OC2Init
                0x0000000008008e14       0x90 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008008e14                TIM_OC2Init
 .text.TIM_OC3Init
                0x0000000008008ea4       0x90 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008008ea4                TIM_OC3Init
 .text.TIM_ICInit
                0x0000000008008f34      0x180 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008008f34                TIM_ICInit
 .text.TIM_Cmd  0x00000000080090b4       0x1c lib/libstm32.a(stm32f10x_tim.o)
                0x00000000080090b4                TIM_Cmd
 .text.TIM_CtrlPWMOutputs
                0x00000000080090d0       0x20 lib/libstm32.a(stm32f10x_tim.o)
                0x00000000080090d0                TIM_CtrlPWMOutputs
 .text.TIM_ITConfig
                0x00000000080090f0       0x14 lib/libstm32.a(stm32f10x_tim.o)
                0x00000000080090f0                TIM_ITConfig
 .text.TIM_SelectInputTrigger
                0x0000000008009104       0x10 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009104                TIM_SelectInputTrigger
 .text.TIM_OC1PreloadConfig
                0x0000000008009114       0x10 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009114                TIM_OC1PreloadConfig
 .text.TIM_OC3PreloadConfig
                0x0000000008009124       0x10 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009124                TIM_OC3PreloadConfig
 .text.TIM_OC1PolarityConfig
                0x0000000008009134       0x10 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009134                TIM_OC1PolarityConfig
 .text.TIM_OC3PolarityConfig
                0x0000000008009144       0x14 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009144                TIM_OC3PolarityConfig
 .text.TIM_SelectSlaveMode
                0x0000000008009158       0x18 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009158                TIM_SelectSlaveMode
 .text.TIM_SetCounter
                0x0000000008009170        0x4 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009170                TIM_SetCounter
 .text.TIM_SetCompare1
                0x0000000008009174        0x4 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009174                TIM_SetCompare1
 .text.TIM_SetCompare3
                0x0000000008009178        0x4 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009178                TIM_SetCompare3
 .text.TIM_GetCounter
                0x000000000800917c        0x8 lib/libstm32.a(stm32f10x_tim.o)
                0x000000000800917c                TIM_GetCounter
 .text.TIM_GetITStatus
                0x0000000008009184       0x18 lib/libstm32.a(stm32f10x_tim.o)
                0x0000000008009184                TIM_GetITStatus
 .text.TIM_ClearITPendingBit
                0x000000000800919c        0x8 lib/libstm32.a(stm32f10x_tim.o)
                0x000000000800919c                TIM_ClearITPendingBit
 .text.ADC_Init
                0x00000000080091a4       0x4c lib/libstm32.a(stm32f10x_adc.o)
                0x00000000080091a4                ADC_Init
 .text.ADC_Cmd  0x00000000080091f0       0x14 lib/libstm32.a(stm32f10x_adc.o)
                0x00000000080091f0                ADC_Cmd
 .text.ADC_DMACmd
                0x0000000008009204       0x14 lib/libstm32.a(stm32f10x_adc.o)
                0x0000000008009204                ADC_DMACmd
 .text.ADC_ResetCalibration
                0x0000000008009218        0xc lib/libstm32.a(stm32f10x_adc.o)
                0x0000000008009218                ADC_ResetCalibration
 .text.ADC_GetResetCalibrationStatus
                0x0000000008009224        0x8 lib/libstm32.a(stm32f10x_adc.o)
                0x0000000008009224                ADC_GetResetCalibrationStatus
 .text.ADC_StartCalibration
                0x000000000800922c        0xc lib/libstm32.a(stm32f10x_adc.o)
                0x000000000800922c                ADC_StartCalibration
 .text.ADC_GetCalibrationStatus
                0x0000000008009238        0x8 lib/libstm32.a(stm32f10x_adc.o)
                0x0000000008009238                ADC_GetCalibrationStatus
 .text.ADC_SoftwareStartConvCmd
                0x0000000008009240       0x14 lib/libstm32.a(stm32f10x_adc.o)
                0x0000000008009240                ADC_SoftwareStartConvCmd
 .text.ADC_RegularChannelConfig
                0x0000000008009254       0x9c lib/libstm32.a(stm32f10x_adc.o)
                0x0000000008009254                ADC_RegularChannelConfig
 .text.ADC_ExternalTrigConvCmd
                0x00000000080092f0       0x14 lib/libstm32.a(stm32f10x_adc.o)
                0x00000000080092f0                ADC_ExternalTrigConvCmd
 .text.ADC_TempSensorVrefintCmd
                0x0000000008009304       0x1c lib/libstm32.a(stm32f10x_adc.o)
                0x0000000008009304                ADC_TempSensorVrefintCmd
 .text.USART_Init
                0x0000000008009320       0xb8 lib/libstm32.a(stm32f10x_usart.o)
                0x0000000008009320                USART_Init
 .text.USART_Cmd
                0x00000000080093d8       0x1c lib/libstm32.a(stm32f10x_usart.o)
                0x00000000080093d8                USART_Cmd
 .text.USART_ITConfig
                0x00000000080093f4       0x34 lib/libstm32.a(stm32f10x_usart.o)
                0x00000000080093f4                USART_ITConfig
 .text.USART_SetAddress
                0x0000000008009428       0x18 lib/libstm32.a(stm32f10x_usart.o)
                0x0000000008009428                USART_SetAddress
 .text.USART_WakeUpConfig
                0x0000000008009440       0x18 lib/libstm32.a(stm32f10x_usart.o)
                0x0000000008009440                USART_WakeUpConfig
 .text.USART_SendData
                0x0000000008009458        0x8 lib/libstm32.a(stm32f10x_usart.o)
                0x0000000008009458                USART_SendData
 .text.USART_ReceiveData
                0x0000000008009460        0x8 lib/libstm32.a(stm32f10x_usart.o)
                0x0000000008009460                USART_ReceiveData
 .text.USART_GetFlagStatus
                0x0000000008009468        0xc lib/libstm32.a(stm32f10x_usart.o)
                0x0000000008009468                USART_GetFlagStatus
 .text.USART_ClearFlag
                0x0000000008009474        0x8 lib/libstm32.a(stm32f10x_usart.o)
                0x0000000008009474                USART_ClearFlag
 .text.PWR_PVDCmd
                0x000000000800947c        0xc lib/libstm32.a(stm32f10x_pwr.o)
                0x000000000800947c                PWR_PVDCmd
 .text.PWR_PVDLevelConfig
                0x0000000008009488       0x14 lib/libstm32.a(stm32f10x_pwr.o)
                0x0000000008009488                PWR_PVDLevelConfig
 .text.PWR_GetFlagStatus
                0x000000000800949c       0x14 lib/libstm32.a(stm32f10x_pwr.o)
                0x000000000800949c                PWR_GetFlagStatus
 .text.DMA_DeInit
                0x00000000080094b0      0x128 lib/libstm32.a(stm32f10x_dma.o)
                0x00000000080094b0                DMA_DeInit
 .text.DMA_Init
                0x00000000080095d8       0x40 lib/libstm32.a(stm32f10x_dma.o)
                0x00000000080095d8                DMA_Init
 .text.DMA_Cmd  0x0000000008009618       0x18 lib/libstm32.a(stm32f10x_dma.o)
                0x0000000008009618                DMA_Cmd
 .text.atan2    0x0000000008009630        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o)
                0x0000000008009630                atan2
 *fill*         0x0000000008009634        0x4 
 .text.__ieee754_atan2
                0x0000000008009638      0x194 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
                0x0000000008009638                __ieee754_atan2
 *fill*         0x00000000080097cc        0x4 
 .text.atan     0x00000000080097d0      0x31c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
                0x00000000080097d0                atan
 .text.fabs     0x0000000008009aec        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_fabs.o)
                0x0000000008009aec                fabs
 .text.atoi     0x0000000008009af4        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
                0x0000000008009af4                atoi
 .text.atol     0x0000000008009afc        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
                0x0000000008009afc                atol
 .text.__errno  0x0000000008009b04        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-errno.o)
                0x0000000008009b04                __errno
 .text.__sflush_r
                0x0000000008009b10      0x104 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
                0x0000000008009b10                __sflush_r
 .text._fflush_r
                0x0000000008009c14       0x78 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
                0x0000000008009c14                _fflush_r
 .text.fflush   0x0000000008009c8c       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
                0x0000000008009c8c                fflush
 .text.fileno   0x0000000008009cb0       0x7c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fileno.o)
                0x0000000008009cb0                fileno
 .text.std      0x0000000008009d2c       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .text._cleanup_r
                0x0000000008009d74        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009d74                _cleanup_r
 .text.__sfmoreglue
                0x0000000008009d80       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009d80                __sfmoreglue
 .text.__sfp_lock_acquire
                0x0000000008009dac        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009dac                __sfp_lock_acquire
 .text.__sfp_lock_release
                0x0000000008009db8        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009db8                __sfp_lock_release
 .text.__sinit_lock_acquire
                0x0000000008009dc4        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009dc4                __sinit_lock_acquire
 .text.__sinit_lock_release
                0x0000000008009dd0        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009dd0                __sinit_lock_release
 .text.__sinit  0x0000000008009ddc       0x70 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009ddc                __sinit
 .text.__sfp    0x0000000008009e4c       0x8c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008009e4c                __sfp
 .text._fwalk   0x0000000008009ed8       0x3a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
                0x0000000008009ed8                _fwalk
 .text._fwalk_reent
                0x0000000008009f12       0x3e /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
                0x0000000008009f12                _fwalk_reent
 .text.getchar  0x0000000008009f50       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
                0x0000000008009f50                getchar
 .text.__retarget_lock_init_recursive
                0x0000000008009f74        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
                0x0000000008009f74                __retarget_lock_init_recursive
 .text.__retarget_lock_acquire_recursive
                0x0000000008009f76        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
                0x0000000008009f76                __retarget_lock_acquire_recursive
 .text.__retarget_lock_release_recursive
                0x0000000008009f78        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
                0x0000000008009f78                __retarget_lock_release_recursive
 .text.memcpy   0x0000000008009f7a       0x1c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memcpy-stub.o)
                0x0000000008009f7a                memcpy
 .text.memset   0x0000000008009f96       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memset.o)
                0x0000000008009f96                memset
 .text.__cvt    0x0000000008009fa6       0xbe /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
                0x0000000008009fa6                __cvt
 .text.__exponent
                0x000000000800a064       0x7c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
                0x000000000800a064                __exponent
 .text._printf_float
                0x000000000800a0e0      0x458 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
                0x000000000800a0e0                _printf_float
 .text._printf_common
                0x000000000800a538       0xde /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
                0x000000000800a538                _printf_common
 *fill*         0x000000000800a616        0x2 
 .text._printf_i
                0x000000000800a618      0x24c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
                0x000000000800a618                _printf_i
 .text.printf   0x000000000800a864       0x30 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
                0x000000000800a864                printf
                0x000000000800a864                iprintf
 .text.cleanup_glue
                0x000000000800a894       0x1a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
                0x000000000800a894                cleanup_glue
 *fill*         0x000000000800a8ae        0x2 
 .text._reclaim_reent
                0x000000000800a8b0       0xb8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
                0x000000000800a8b0                _reclaim_reent
 .text.scanf    0x000000000800a968       0x30 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
                0x000000000800a968                iscanf
                0x000000000800a968                scanf
 .text.setvbuf  0x000000000800a998      0x18c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o)
                0x000000000800a998                setvbuf
 .text.snprintf
                0x000000000800ab24       0x68 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
                0x000000000800ab24                snprintf
                0x000000000800ab24                sniprintf
 .text.__sread  0x000000000800ab8c       0x22 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
                0x000000000800ab8c                __sread
 .text.__swrite
                0x000000000800abae       0x38 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
                0x000000000800abae                __swrite
 .text.__sseek  0x000000000800abe6       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
                0x000000000800abe6                __sseek
 .text.__sclose
                0x000000000800ac0a        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
                0x000000000800ac0a                __sclose
 .text.strcat   0x000000000800ac12       0x1e /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcat.o)
                0x000000000800ac12                strcat
 .text.strchr   0x000000000800ac30       0x1a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strchr.o)
                0x000000000800ac30                strchr
 .text.strncmp  0x000000000800ac4a       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncmp.o)
                0x000000000800ac4a                strncmp
 .text.strncpy  0x000000000800ac72       0x26 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncpy.o)
                0x000000000800ac72                strncpy
 .text._strtol_l.constprop.0
                0x000000000800ac98       0xf8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
 .text._strtol_r
                0x000000000800ad90        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
                0x000000000800ad90                _strtol_r
 .text.strtol   0x000000000800ad94       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
                0x000000000800ad94                strtol
 .text.write    0x000000000800ada8       0x14 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-syswrite.o)
                0x000000000800ada8                write
 .text._write_r
                0x000000000800adbc       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-writer.o)
                0x000000000800adbc                _write_r
 .text._close_r
                0x000000000800ade0       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-closer.o)
                0x000000000800ade0                _close_r
 .text.quorem   0x000000000800ae00      0x118 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
 .text._dtoa_r  0x000000000800af18      0xbdc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
                0x000000000800af18                _dtoa_r
 .text._getc_r  0x000000000800baf4       0x80 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
                0x000000000800baf4                _getc_r
 .text._localeconv_r
                0x000000000800bb74        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
                0x000000000800bb74                _localeconv_r
 .text._lseek_r
                0x000000000800bb7c       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lseekr.o)
                0x000000000800bb7c                _lseek_r
 .text.__swhatbuf_r
                0x000000000800bba0       0x4a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
                0x000000000800bba0                __swhatbuf_r
 *fill*         0x000000000800bbea        0x2 
 .text.__smakebuf_r
                0x000000000800bbec       0x80 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
                0x000000000800bbec                __smakebuf_r
 .text.malloc   0x000000000800bc6c       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
                0x000000000800bc6c                malloc
 .text.memchr   0x000000000800bc7c       0x1c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memchr-stub.o)
                0x000000000800bc7c                memchr
 .text._Balloc  0x000000000800bc98       0x80 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800bc98                _Balloc
 .text._Bfree   0x000000000800bd18       0x44 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800bd18                _Bfree
 .text.__multadd
                0x000000000800bd5c       0x8c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800bd5c                __multadd
 .text.__hi0bits
                0x000000000800bde8       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800bde8                __hi0bits
 .text.__lo0bits
                0x000000000800be28       0x5a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800be28                __lo0bits
 *fill*         0x000000000800be82        0x2 
 .text.__i2b    0x000000000800be84       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800be84                __i2b
 .text.__multiply
                0x000000000800beb0      0x150 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800beb0                __multiply
 .text.__pow5mult
                0x000000000800c000       0xb4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800c000                __pow5mult
 .text.__lshift
                0x000000000800c0b4       0xe0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800c0b4                __lshift
 .text.__mcmp   0x000000000800c194       0x36 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800c194                __mcmp
 *fill*         0x000000000800c1ca        0x2 
 .text.__mdiff  0x000000000800c1cc      0x118 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800c1cc                __mdiff
 .text.__d2b    0x000000000800c2e4       0xb8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x000000000800c2e4                __d2b
 .text.__ssputs_r
                0x000000000800c39c       0xb6 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
                0x000000000800c39c                __ssputs_r
 *fill*         0x000000000800c452        0x2 
 .text._svfprintf_r
                0x000000000800c454      0x200 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
                0x000000000800c454                _svfprintf_r
                0x000000000800c454                _svfiprintf_r
 .text.__sfputc_r
                0x000000000800c654       0x2a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .text.__sfputs_r
                0x000000000800c67e       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
                0x000000000800c67e                __sfputs_r
 *fill*         0x000000000800c6a2        0x2 
 .text._vfprintf_r
                0x000000000800c6a4      0x260 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
                0x000000000800c6a4                _vfiprintf_r
                0x000000000800c6a4                _vfprintf_r
 .text.__svfscanf_r
                0x000000000800c904      0x348 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
                0x000000000800c904                __svfiscanf_r
                0x000000000800c904                __svfscanf_r
 .text._vfscanf_r
                0x000000000800cc4c       0x50 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
                0x000000000800cc4c                _vfiscanf_r
                0x000000000800cc4c                _vfscanf_r
 .text._scanf_chars
                0x000000000800cc9c       0xb4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
                0x000000000800cc9c                _scanf_chars
 .text._scanf_i
                0x000000000800cd50      0x1e4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
                0x000000000800cd50                _scanf_i
 .text._read_r  0x000000000800cf34       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-readr.o)
                0x000000000800cf34                _read_r
 .text.lflush   0x000000000800cf58       0x12 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
 *fill*         0x000000000800cf6a        0x2 
 .text.__srefill_r
                0x000000000800cf6c      0x118 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
                0x000000000800cf6c                __srefill_r
 .text.__srget_r
                0x000000000800d084       0x58 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
                0x000000000800d084                __srget_r
 .text.__sccl   0x000000000800d0dc       0x70 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-sccl.o)
                0x000000000800d0dc                __sccl
 .text._strtoul_l.constprop.0
                0x000000000800d14c       0xe4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .text._strtoul_r
                0x000000000800d230        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
                0x000000000800d230                _strtoul_r
 .text.__submore
                0x000000000800d234       0x70 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
                0x000000000800d234                __submore
 .text._ungetc_r
                0x000000000800d2a4      0x130 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
                0x000000000800d2a4                _ungetc_r
 .text.__swbuf_r
                0x000000000800d3d4       0xa4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
                0x000000000800d3d4                __swbuf_r
 .text.__swsetup_r
                0x000000000800d478       0xdc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wsetup.o)
                0x000000000800d478                __swsetup_r
 .text.__assert_func
                0x000000000800d554       0x3c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
                0x000000000800d554                __assert_func
 .text.fprintf  0x000000000800d590       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
                0x000000000800d590                fprintf
                0x000000000800d590                fiprintf
 .text._fstat_r
                0x000000000800d5b4       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fstatr.o)
                0x000000000800d5b4                _fstat_r
 .text._isatty_r
                0x000000000800d5d8       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-isattyr.o)
                0x000000000800d5d8                _isatty_r
 .text.__ascii_mbtowc
                0x000000000800d5f8       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
                0x000000000800d5f8                __ascii_mbtowc
 .text.memmove  0x000000000800d61c       0x34 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memmove.o)
                0x000000000800d61c                memmove
 .text.__ascii_wctomb
                0x000000000800d650       0x1a /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
                0x000000000800d650                __ascii_wctomb
 .text.abort    0x000000000800d66a        0xe /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o)
                0x000000000800d66a                abort
 .text._raise_r
                0x000000000800d678       0x50 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
                0x000000000800d678                _raise_r
 .text.raise    0x000000000800d6c8       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
                0x000000000800d6c8                raise
 .text._kill_r  0x000000000800d6d8       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
                0x000000000800d6d8                _kill_r
 .text._getpid_r
                0x000000000800d6fc        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
                0x000000000800d6fc                _getpid_r
 *(.rodata)
 .rodata        0x000000000800d700       0x18 obj/mod_lcd.o
 .rodata        0x000000000800d718        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 *(.rodata*)
 .rodata.vSound_Signal_RF_Control.str1.4
                0x000000000800d724       0x21 obj/mod_sound.o
 *fill*         0x000000000800d745        0x3 
 .rodata.lcd_orient_display_state.str1.4
                0x000000000800d748       0x26 obj/mod_lcd.o
 *fill*         0x000000000800d76e        0x2 
 .rodata.lcd_status_display_options.str1.4
                0x000000000800d770       0x2f obj/mod_lcd.o
 *fill*         0x000000000800d79f        0x1 
 .rodata.lcd_light_display_options.str1.4
                0x000000000800d7a0       0x3d obj/mod_lcd.o
 *fill*         0x000000000800d7dd        0x3 
 .rodata.lcd_status_on_confirm.str1.4
                0x000000000800d7e0       0x39 obj/mod_lcd.o
 *fill*         0x000000000800d819        0x3 
 .rodata.lcd_enviro_display_state.str1.4
                0x000000000800d81c       0x46 obj/mod_lcd.o
 *fill*         0x000000000800d862        0x2 
 .rodata.lcd_status_display_state.str1.4
                0x000000000800d864       0xb9 obj/mod_lcd.o
 *fill*         0x000000000800d91d        0x3 
 .rodata.lcd_mov_display_state.str1.4
                0x000000000800d920       0x1d obj/mod_lcd.o
 *fill*         0x000000000800d93d        0x3 
 .rodata.vLcdTask.str1.4
                0x000000000800d940       0x75 obj/mod_lcd.o
 *fill*         0x000000000800d9b5        0x3 
 .rodata.str1.4
                0x000000000800d9b8      0x1e1 obj/mod_lcd.o
 *fill*         0x000000000800db99        0x3 
 .rodata.CSWTCH.77
                0x000000000800db9c       0x28 obj/mod_lcd.o
 .rodata.CSWTCH.92
                0x000000000800dbc4       0x3c obj/mod_lcd.o
 .rodata.battery_full
                0x000000000800dc00       0x30 obj/mod_lcd.o
 .rodata.battery_half
                0x000000000800dc30       0x30 obj/mod_lcd.o
 .rodata.battery_low
                0x000000000800dc60       0x30 obj/mod_lcd.o
 .rodata.buzzer_off
                0x000000000800dc90      0x180 obj/mod_lcd.o
 .rodata.buzzer_on
                0x000000000800de10      0x180 obj/mod_lcd.o
 .rodata.degree_symbol
                0x000000000800df90       0x10 obj/mod_lcd.o
 .rodata.dirInfo.2
                0x000000000800dfa0       0x28 obj/mod_lcd.o
 .rodata.enviromenu
                0x000000000800dfc8       0x20 obj/mod_lcd.o
 .rodata.exitmenu
                0x000000000800dfe8       0x20 obj/mod_lcd.o
 .rodata.lighting_all
                0x000000000800e008      0x140 obj/mod_lcd.o
 .rodata.lighting_inner
                0x000000000800e148      0x140 obj/mod_lcd.o
 .rodata.lighting_inner_outer
                0x000000000800e288      0x140 obj/mod_lcd.o
 .rodata.lighting_left
                0x000000000800e3c8      0x140 obj/mod_lcd.o
 .rodata.lighting_left_inner
                0x000000000800e508      0x140 obj/mod_lcd.o
 .rodata.lighting_left_inner_outer
                0x000000000800e648      0x140 obj/mod_lcd.o
 .rodata.lighting_left_outer
                0x000000000800e788      0x140 obj/mod_lcd.o
 .rodata.lighting_left_right
                0x000000000800e8c8      0x140 obj/mod_lcd.o
 .rodata.lighting_left_right_inner
                0x000000000800ea08      0x140 obj/mod_lcd.o
 .rodata.lighting_left_right_outer
                0x000000000800eb48      0x140 obj/mod_lcd.o
 .rodata.lighting_none
                0x000000000800ec88      0x140 obj/mod_lcd.o
 .rodata.lighting_outer
                0x000000000800edc8      0x140 obj/mod_lcd.o
 .rodata.lighting_right
                0x000000000800ef08      0x140 obj/mod_lcd.o
 .rodata.lighting_right_inner
                0x000000000800f048      0x140 obj/mod_lcd.o
 .rodata.lighting_right_inner_outer
                0x000000000800f188      0x140 obj/mod_lcd.o
 .rodata.lighting_right_outer
                0x000000000800f2c8      0x140 obj/mod_lcd.o
 .rodata.ligmenu
                0x000000000800f408       0x20 obj/mod_lcd.o
 .rodata.mov_backward
                0x000000000800f428      0x168 obj/mod_lcd.o
 .rodata.mov_forward
                0x000000000800f590      0x168 obj/mod_lcd.o
 .rodata.mov_left
                0x000000000800f6f8      0x168 obj/mod_lcd.o
 .rodata.mov_right
                0x000000000800f860      0x168 obj/mod_lcd.o
 .rodata.mov_stopped
                0x000000000800f9c8      0x168 obj/mod_lcd.o
                0x000000000800f9c8                mov_stopped
 .rodata.movmenu
                0x000000000800fb30       0x20 obj/mod_lcd.o
 .rodata.orientmenu
                0x000000000800fb50       0x20 obj/mod_lcd.o
 .rodata.robot_hand_down
                0x000000000800fb70      0x200 obj/mod_lcd.o
 .rodata.robot_hand_up
                0x000000000800fd70      0x200 obj/mod_lcd.o
 .rodata.sigmenu
                0x000000000800ff70       0x20 obj/mod_lcd.o
 .rodata.statusmenu
                0x000000000800ff90       0x20 obj/mod_lcd.o
 .rodata.vLight_mode.str1.4
                0x000000000800ffb0       0x27 obj/mod_console.o
 *fill*         0x000000000800ffd7        0x1 
 .rodata.vDrive_speed.str1.4
                0x000000000800ffd8       0x6f obj/mod_console.o
 *fill*         0x0000000008010047        0x1 
 .rodata.console_print_menu_header.str1.4
                0x0000000008010048       0x77 obj/mod_console.o
                                         0x7b (size before relaxing)
 *fill*         0x00000000080100bf        0x1 
 .rodata.console_print_sub_menu.str1.4
                0x00000000080100c0       0x54 obj/mod_console.o
                                         0x74 (size before relaxing)
 .rodata.vLight_setup.str1.4
                0x0000000008010114       0x1d obj/mod_console.o
 *fill*         0x0000000008010131        0x3 
 .rodata.vSound_mode.str1.4
                0x0000000008010134        0xe obj/mod_console.o
 *fill*         0x0000000008010142        0x2 
 .rodata.vConsoleTask.str1.4
                0x0000000008010144       0x28 obj/mod_console.o
 .rodata.str1.4
                0x000000000801016c       0xee obj/mod_console.o
 *fill*         0x000000000801025a        0x2 
 .rodata.radio_recv_handler.str1.4
                0x000000000801025c        0x6 obj/mod_radio.o
 *fill*         0x0000000008010262        0x2 
 .rodata.radio_send_command.str1.4
                0x0000000008010264        0xc obj/mod_radio.o
 .rodata.radio_get_config.str1.4
                0x0000000008010270       0x1d obj/mod_radio.o
 *fill*         0x000000000801028d        0x3 
 .rodata.radio_get_payload.str1.4
                0x0000000008010290       0x16 obj/mod_radio.o
 *fill*         0x00000000080102a6        0x2 
 .rodata.radio_get_rfdata_rate.str1.4
                0x00000000080102a8        0xc obj/mod_radio.o
 .rodata.radio_get_uart_baudrate.str1.4
                0x00000000080102b4        0xd obj/mod_radio.o
 *fill*         0x00000000080102c1        0x3 
 .rodata.radio_get_txrx_led.str1.4
                0x00000000080102c4       0x1e obj/mod_radio.o
 *fill*         0x00000000080102e2        0x2 
 .rodata.radio_get_modulation.str1.4
                0x00000000080102e4        0xe obj/mod_radio.o
 *fill*         0x00000000080102f2        0x2 
 .rodata.radio_get_freq.str1.4
                0x00000000080102f4       0x1c obj/mod_radio.o
 .rodata.print_results.str1.4
                0x0000000008010310        0x8 obj/mod_radio.o
 .rodata.vRadioTask.str1.4
                0x0000000008010318       0x62 obj/mod_radio.o
 *fill*         0x000000000801037a        0x2 
 .rodata.str1.4
                0x000000000801037c       0x49 obj/mod_radio.o
 *fill*         0x00000000080103c5        0x3 
 .rodata.CSWTCH.34
                0x00000000080103c8        0xc obj/mod_radio.o
 .rodata.CSWTCH.36
                0x00000000080103d4       0x18 obj/mod_radio.o
 .rodata.__func__.4
                0x00000000080103ec        0xb obj/mod_radio.o
 *fill*         0x00000000080103f7        0x1 
 .rodata.radio_init_cfgs
                0x00000000080103f8       0x18 obj/mod_radio.o
 .rodata.TIM4_IRQHandler.str1.4
                0x0000000008010410        0x8 /tmp/cchESTC9.o
 .rodata.main.str1.4
                0x0000000008010418       0x18 /tmp/cc9a8G1U.o
                                         0x28 (size before relaxing)
 .rodata.vApplicationStackOverflowHook.str1.4
                0x0000000008010430       0x13 /tmp/cc9a8G1U.o
 *fill*         0x0000000008010443        0x1 
 .rodata.Virtual_Com_Port_ConfigDescriptor
                0x0000000008010444       0x43 lib/libusb.a(usb_desc.o)
                0x0000000008010444                Virtual_Com_Port_ConfigDescriptor
 *fill*         0x0000000008010487        0x1 
 .rodata.Virtual_Com_Port_DeviceDescriptor
                0x0000000008010488       0x12 lib/libusb.a(usb_desc.o)
                0x0000000008010488                Virtual_Com_Port_DeviceDescriptor
 *fill*         0x000000000801049a        0x2 
 .rodata.Virtual_Com_Port_StringLangID
                0x000000000801049c        0x4 lib/libusb.a(usb_desc.o)
                0x000000000801049c                Virtual_Com_Port_StringLangID
 .rodata.Virtual_Com_Port_StringProduct
                0x00000000080104a0       0x32 lib/libusb.a(usb_desc.o)
                0x00000000080104a0                Virtual_Com_Port_StringProduct
 *fill*         0x00000000080104d2        0x2 
 .rodata.Virtual_Com_Port_StringVendor
                0x00000000080104d4       0x26 lib/libusb.a(usb_desc.o)
                0x00000000080104d4                Virtual_Com_Port_StringVendor
 *fill*         0x00000000080104fa        0x2 
 .rodata.vTaskStartScheduler.str1.4
                0x00000000080104fc        0x5 lib/libfreertos.a(tasks.o)
 *fill*         0x0000000008010501        0x3 
 .rodata.xTimerCreateTimerTask.str1.4
                0x0000000008010504        0x8 lib/libfreertos.a(timers.o)
 *fill*         0x000000000801050c        0x4 
 .rodata.CSWTCH.8
                0x0000000008010510       0x18 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
 .rodata.CSWTCH.9
                0x0000000008010528       0x18 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
 .rodata.atanhi
                0x0000000008010540       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
 .rodata.atanlo
                0x0000000008010560       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
 .rodata.__sf_fake_stderr
                0x0000000008010580       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x0000000008010580                __sf_fake_stderr
 .rodata.__sf_fake_stdin
                0x00000000080105a0       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x00000000080105a0                __sf_fake_stdin
 .rodata.__sf_fake_stdout
                0x00000000080105c0       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
                0x00000000080105c0                __sf_fake_stdout
 .rodata._global_impure_ptr
                0x00000000080105e0        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
                0x00000000080105e0                _global_impure_ptr
 .rodata._printf_float.str1.1
                0x00000000080105e4       0x10 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
                                         0x12 (size before relaxing)
 .rodata._printf_i.str1.1
                0x00000000080105f4       0x22 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
 .rodata._ctype_
                0x0000000008010616      0x101 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ctype_.o)
                0x0000000008010616                _ctype_
 .rodata._dtoa_r.str1.1
                0x0000000008010717       0xb2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
                                         0xb4 (size before relaxing)
 .rodata._Balloc.str1.1
                0x00000000080107c9       0x7e /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                                         0x95 (size before relaxing)
 .rodata.__multadd.str1.1
                0x0000000008010847       0x11 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 *fill*         0x0000000008010847        0x1 
 .rodata.__mprec_bigtens
                0x0000000008010848       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x0000000008010848                __mprec_bigtens
 .rodata.__mprec_tens
                0x0000000008010870       0xc8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
                0x0000000008010870                __mprec_tens
 .rodata.p05.0  0x0000000008010938        0xc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .rodata._svfprintf_r.str1.1
                0x0000000008010944       0x11 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
 .rodata._vfprintf_r.str1.1
                0x0000000008010955       0x11 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .rodata.__svfscanf_r.str1.1
                0x0000000008010955        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .rodata.str1.1
                0x0000000008010955        0x9 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 .rodata._scanf_i.str1.1
                0x000000000801095e       0x12 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 .rodata.__assert_func.str1.1
                0x0000000008010970       0x3c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
                                         0x3d (size before relaxing)
 .rodata._setlocale_r.str1.1
                0x00000000080109ac        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
                                          0x9 (size before relaxing)
 .rodata.str1.1
                0x00000000080109b4        0x2 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 *(.glue_7)
 .glue_7        0x00000000080109b6        0x0 linker stubs
 *(.glue_7t)
 .glue_7t       0x00000000080109b6        0x0 linker stubs
 *(.data.__global_locale)
 *fill*         0x00000000080109b6        0x2 
 .data.__global_locale
                0x00000000080109b8      0x16c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
                0x00000000080109b8                __global_locale
 *(.init)
 .init          0x0000000008010b24        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crti.o
                0x0000000008010b24                _init
 .init          0x0000000008010b28        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtn.o
 *(.fini)
 .fini          0x0000000008010b30        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crti.o
                0x0000000008010b30                _fini
 .fini          0x0000000008010b34        0x8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtn.o
                0x0000000008010b3c                . = ALIGN (0x4)
                0x0000000008010b3c                _etext = .

.vfp11_veneer   0x0000000008010b3c        0x0
 .vfp11_veneer  0x0000000008010b3c        0x0 linker stubs

.v4_bx          0x0000000008010b3c        0x0
 .v4_bx         0x0000000008010b3c        0x0 linker stubs

.iplt           0x0000000008010b3c        0x0
 .iplt          0x0000000008010b3c        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o

.rel.dyn        0x0000000008010b3c        0x0
 .rel.iplt      0x0000000008010b3c        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o

.ARM.extab
 *(.ARM.extab* .gnu.linkonce.armextab.*)

.ARM            0x0000000008010b3c        0x0
                0x0000000008010b3c                __exidx_start = .
 *(.ARM.exidx*)
                0x0000000008010b3c                __exidx_end = .

.ARM.attributes
                0x0000000008010b3c       0x29
 *(.ARM.attributes)
 .ARM.attributes
                0x0000000008010b3c       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crti.o
 .ARM.attributes
                0x0000000008010b59       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
 .ARM.attributes
                0x0000000008010b86       0x2d obj/mod_led.o
 .ARM.attributes
                0x0000000008010bb3       0x2d obj/mod_sound.o
 .ARM.attributes
                0x0000000008010be0       0x2d obj/mod_sensors.o
 .ARM.attributes
                0x0000000008010c0d       0x2d obj/mod_drive.o
 .ARM.attributes
                0x0000000008010c3a       0x2d obj/mod_orientsensor.o
 .ARM.attributes
                0x0000000008010c67       0x2d obj/mod_lcd.o
 .ARM.attributes
                0x0000000008010c94       0x2d obj/mod_lighting.o
 .ARM.attributes
                0x0000000008010cc1       0x2d obj/mod_console.o
 .ARM.attributes
                0x0000000008010cee       0x2d obj/mod_i2c.o
 .ARM.attributes
                0x0000000008010d1b       0x2d obj/mod_radio.o
 .ARM.attributes
                0x0000000008010d48       0x2d obj/mod_auxiliary.o
 .ARM.attributes
                0x0000000008010d75       0x2d /tmp/ccPh1D7g.o
 .ARM.attributes
                0x0000000008010da2       0x2d /tmp/cchESTC9.o
 .ARM.attributes
                0x0000000008010dcf       0x2d /tmp/cc9ErQe2.o
 .ARM.attributes
                0x0000000008010dfc       0x2d /tmp/cc9a8G1U.o
 .ARM.attributes
                0x0000000008010e29       0x2d /tmp/ccTyJAXN.o
 .ARM.attributes
                0x0000000008010e56       0x2d /tmp/ccrbnG3G.o
 .ARM.attributes
                0x0000000008010e83       0x21 /tmp/ccJdFGaA.o
 .ARM.attributes
                0x0000000008010ea4       0x2d lib/libusb.a(hw_config.o)
 .ARM.attributes
                0x0000000008010ed1       0x33 lib/libusb.a(usb_desc.o)
 .ARM.attributes
                0x0000000008010f04       0x2d lib/libusb.a(usb_endp.o)
 .ARM.attributes
                0x0000000008010f31       0x2d lib/libusb.a(usb_istr.o)
 .ARM.attributes
                0x0000000008010f5e       0x2d lib/libusb.a(usb_prop.o)
 .ARM.attributes
                0x0000000008010f8b       0x2d lib/libusb.a(usb_pwr.o)
 .ARM.attributes
                0x0000000008010fb8       0x2d lib/libusb.a(usb_core.o)
 .ARM.attributes
                0x0000000008010fe5       0x2d lib/libusb.a(usb_init.o)
 .ARM.attributes
                0x0000000008011012       0x2d lib/libusb.a(usb_int.o)
 .ARM.attributes
                0x000000000801103f       0x2d lib/libusb.a(usb_mem.o)
 .ARM.attributes
                0x000000000801106c       0x2d lib/libusb.a(usb_regs.o)
 .ARM.attributes
                0x0000000008011099       0x2d lib/libusb.a(usb_sil.o)
 .ARM.attributes
                0x00000000080110c6       0x2d lib/libfreertos.a(tasks.o)
 .ARM.attributes
                0x00000000080110f3       0x2d lib/libfreertos.a(list.o)
 .ARM.attributes
                0x0000000008011120       0x2d lib/libfreertos.a(queue.o)
 .ARM.attributes
                0x000000000801114d       0x2d lib/libfreertos.a(timers.o)
 .ARM.attributes
                0x000000000801117a       0x2d lib/libfreertos.a(port.o)
 .ARM.attributes
                0x00000000080111a7       0x2d lib/libfreertos.a(heap_4.o)
 .ARM.attributes
                0x00000000080111d4       0x2d lib/libstm32.a(misc.o)
 .ARM.attributes
                0x0000000008011201       0x2d lib/libstm32.a(stm32f10x_rcc.o)
 .ARM.attributes
                0x000000000801122e       0x2d lib/libstm32.a(stm32f10x_gpio.o)
 .ARM.attributes
                0x000000000801125b       0x2d lib/libstm32.a(stm32f10x_exti.o)
 .ARM.attributes
                0x0000000008011288       0x2d lib/libstm32.a(stm32f10x_tim.o)
 .ARM.attributes
                0x00000000080112b5       0x2d lib/libstm32.a(stm32f10x_adc.o)
 .ARM.attributes
                0x00000000080112e2       0x2d lib/libstm32.a(stm32f10x_usart.o)
 .ARM.attributes
                0x000000000801130f       0x2d lib/libstm32.a(stm32f10x_pwr.o)
 .ARM.attributes
                0x000000000801133c       0x2d lib/libstm32.a(stm32f10x_dma.o)
 .ARM.attributes
                0x0000000008011369       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o)
 .ARM.attributes
                0x0000000008011396       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
 .ARM.attributes
                0x00000000080113c3       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
 .ARM.attributes
                0x00000000080113f0       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_fabs.o)
 .ARM.attributes
                0x000000000801141d       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubdf3.o)
 .ARM.attributes
                0x000000000801143a       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivdf3.o)
 .ARM.attributes
                0x0000000008011457       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpdf2.o)
 .ARM.attributes
                0x0000000008011474       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixunsdfsi.o)
 .ARM.attributes
                0x0000000008011491       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubsf3.o)
 .ARM.attributes
                0x00000000080114ae       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivsf3.o)
 .ARM.attributes
                0x00000000080114cb       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpsf2.o)
 .ARM.attributes
                0x00000000080114e8       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
 .ARM.attributes
                0x0000000008011515       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
 .ARM.attributes
                0x0000000008011542       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-errno.o)
 .ARM.attributes
                0x000000000801156f       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
 .ARM.attributes
                0x000000000801159c       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fileno.o)
 .ARM.attributes
                0x00000000080115c9       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .ARM.attributes
                0x00000000080115f6       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
 .ARM.attributes
                0x0000000008011623       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
 .ARM.attributes
                0x0000000008011650       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
 .ARM.attributes
                0x000000000801167d       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .ARM.attributes
                0x00000000080116aa       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memcpy-stub.o)
 .ARM.attributes
                0x00000000080116d7       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memset.o)
 .ARM.attributes
                0x0000000008011704       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
 .ARM.attributes
                0x0000000008011731       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
 .ARM.attributes
                0x000000000801175e       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
 .ARM.attributes
                0x000000000801178b       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
 .ARM.attributes
                0x00000000080117b8       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
 .ARM.attributes
                0x00000000080117e5       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o)
 .ARM.attributes
                0x0000000008011812       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
 .ARM.attributes
                0x000000000801183f       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
 .ARM.attributes
                0x000000000801186c       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcat.o)
 .ARM.attributes
                0x0000000008011899       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strchr.o)
 .ARM.attributes
                0x00000000080118c6       0x17 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strlen.o)
 .ARM.attributes
                0x00000000080118dd       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncmp.o)
 .ARM.attributes
                0x000000000801190a       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncpy.o)
 .ARM.attributes
                0x0000000008011937       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
 .ARM.attributes
                0x0000000008011964       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-syswrite.o)
 .ARM.attributes
                0x0000000008011991       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-writer.o)
 .ARM.attributes
                0x00000000080119be       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-closer.o)
 .ARM.attributes
                0x00000000080119eb       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ctype_.o)
 .ARM.attributes
                0x0000000008011a18       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
 .ARM.attributes
                0x0000000008011a45       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
 .ARM.attributes
                0x0000000008011a72       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
 .ARM.attributes
                0x0000000008011a9f       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lseekr.o)
 .ARM.attributes
                0x0000000008011acc       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
 .ARM.attributes
                0x0000000008011af9       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
 .ARM.attributes
                0x0000000008011b26       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memchr-stub.o)
 .ARM.attributes
                0x0000000008011b53       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .ARM.attributes
                0x0000000008011b80       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
 .ARM.attributes
                0x0000000008011bad       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .ARM.attributes
                0x0000000008011bda       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .ARM.attributes
                0x0000000008011c07       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 .ARM.attributes
                0x0000000008011c34       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-readr.o)
 .ARM.attributes
                0x0000000008011c61       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
 .ARM.attributes
                0x0000000008011c8e       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
 .ARM.attributes
                0x0000000008011cbb       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-sccl.o)
 .ARM.attributes
                0x0000000008011ce8       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .ARM.attributes
                0x0000000008011d15       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
 .ARM.attributes
                0x0000000008011d42       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
 .ARM.attributes
                0x0000000008011d6f       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wsetup.o)
 .ARM.attributes
                0x0000000008011d9c       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
 .ARM.attributes
                0x0000000008011dc9       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
 .ARM.attributes
                0x0000000008011df6       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fstatr.o)
 .ARM.attributes
                0x0000000008011e23       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-isattyr.o)
 .ARM.attributes
                0x0000000008011e50       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .ARM.attributes
                0x0000000008011e7d       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
 .ARM.attributes
                0x0000000008011eaa       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memmove.o)
 .ARM.attributes
                0x0000000008011ed7       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
 .ARM.attributes
                0x0000000008011f04       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o)
 .ARM.attributes
                0x0000000008011f31       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .ARM.attributes
                0x0000000008011f5e       0x2d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
 .ARM.attributes
                0x0000000008011f8b       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_unorddf2.o)
 .ARM.attributes
                0x0000000008011fa8       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixdfsi.o)
 .ARM.attributes
                0x0000000008011fc5       0x1d /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtn.o

.preinit_array  0x0000000008010b3c        0x0
                0x0000000008010b3c                PROVIDE (__preinit_array_start = .)
 *(.preinit_array*)
                0x0000000008010b3c                PROVIDE (__preinit_array_end = .)

.init_array     0x0000000008010b3c        0x4
                0x0000000008010b3c                PROVIDE (__init_array_start = .)
 *(SORT_BY_NAME(.init_array.*))
 *(.init_array*)
 .init_array    0x0000000008010b3c        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
                0x0000000008010b40                PROVIDE (__init_array_end = .)

.fini_array     0x0000000008010b40        0x4
                [!provide]                        PROVIDE (__fini_array_start = .)
 *(.fini_array*)
 .fini_array    0x0000000008010b40        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
 *(SORT_BY_NAME(.fini_array.*))
                [!provide]                        PROVIDE (__fini_array_end = .)
                0x0000000008010b44                _sidata = .

.data           0x0000000020000000      0x2b4 load address 0x0000000008010b44
                0x0000000020000000                . = ALIGN (0x4)
                0x0000000020000000                _sdata = .
 *(.data)
 *(.data*)
 .data.__compound_literal.0
                0x0000000020000000       0x20 obj/mod_lcd.o
 .data.data.0   0x0000000020000020        0x1 obj/mod_lcd.o
 .data.data.1   0x0000000020000021        0x1 obj/mod_lcd.o
 .data.lcdShowScrSaver
                0x0000000020000022        0x1 obj/mod_lcd.o
 .data.light_mode.2
                0x0000000020000023        0x1 obj/mod_lighting.o
 .data.__compound_literal.0
                0x0000000020000024       0x54 obj/mod_console.o
 .data.__compound_literal.1
                0x0000000020000078       0x54 obj/mod_console.o
 .data.__compound_literal.2
                0x00000000200000cc       0x30 obj/mod_console.o
 .data.__compound_literal.3
                0x00000000200000fc       0x30 obj/mod_console.o
 .data.ligmenu  0x000000002000012c        0xc obj/mod_console.o
 .data.movmenu  0x0000000020000138        0xc obj/mod_console.o
 .data.sigmenu  0x0000000020000144        0xc obj/mod_console.o
 .data.GPIO_InitStructure.0
                0x0000000020000150        0x4 obj/mod_i2c.o
 .data.Virtual_Com_Port_StringSerial
                0x0000000020000154       0x1a lib/libusb.a(usb_desc.o)
                0x0000000020000154                Virtual_Com_Port_StringSerial
 *fill*         0x000000002000016e        0x2 
 .data.pEpInt_IN
                0x0000000020000170       0x1c lib/libusb.a(usb_istr.o)
                0x0000000020000170                pEpInt_IN
 .data.pEpInt_OUT
                0x000000002000018c       0x1c lib/libusb.a(usb_istr.o)
                0x000000002000018c                pEpInt_OUT
 .data.Config_Descriptor
                0x00000000200001a8        0x8 lib/libusb.a(usb_prop.o)
                0x00000000200001a8                Config_Descriptor
 .data.Device_Descriptor
                0x00000000200001b0        0x8 lib/libusb.a(usb_prop.o)
                0x00000000200001b0                Device_Descriptor
 .data.Device_Property
                0x00000000200001b8       0x30 lib/libusb.a(usb_prop.o)
                0x00000000200001b8                Device_Property
 .data.Device_Table
                0x00000000200001e8        0x2 lib/libusb.a(usb_prop.o)
                0x00000000200001e8                Device_Table
 *fill*         0x00000000200001ea        0x2 
 .data.String_Descriptor
                0x00000000200001ec       0x20 lib/libusb.a(usb_prop.o)
                0x00000000200001ec                String_Descriptor
 .data.User_Standard_Requests
                0x000000002000020c       0x24 lib/libusb.a(usb_prop.o)
                0x000000002000020c                User_Standard_Requests
 .data.linecoding
                0x0000000020000230        0x8 lib/libusb.a(usb_prop.o)
                0x0000000020000230                linecoding
 .data.uxCriticalNesting
                0x0000000020000238        0x4 lib/libfreertos.a(port.o)
 .data.ADCPrescTable
                0x000000002000023c        0x4 lib/libstm32.a(stm32f10x_rcc.o)
 .data.APBAHBPrescTable
                0x0000000020000240       0x10 lib/libstm32.a(stm32f10x_rcc.o)
 .data._impure_ptr
                0x0000000020000250        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
                0x0000000020000250                _impure_ptr
 .data.impure_data
                0x0000000020000254       0x60 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-impure.o)
                0x00000000200002b4                . = ALIGN (0x4)
                0x00000000200002b4                _edata = .

.igot.plt       0x00000000200002b4        0x0 load address 0x0000000008010df8
 .igot.plt      0x00000000200002b4        0x0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
                0x00000000200002b4                . = ALIGN (0x4)

.bss            0x00000000200002b8     0x483c load address 0x0000000008010e00
                0x00000000200002b8                _sbss = .
                0x00000000200002b8                __bss_start__ = _sbss
 *(.bss)
 .bss           0x00000000200002b8       0x1c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/crtbegin.o
 *(.bss*)
 .bss.current_lvl.0
                0x00000000200002d4        0x2 obj/mod_led.o
 *fill*         0x00000000200002d6        0x2 
 .bss.ADC_ConvertedData
                0x00000000200002d8       0x10 obj/mod_sensors.o
                0x00000000200002d8                ADC_ConvertedData
 .bss.distance  0x00000000200002e8       0x10 obj/mod_sensors.o
                0x00000000200002e8                distance
 .bss.sensorInUse
                0x00000000200002f8        0x1 obj/mod_sensors.o
                0x00000000200002f8                sensorInUse
 *fill*         0x00000000200002f9        0x7 
 .bss.acclx.5   0x0000000020000300        0x8 obj/mod_orientsensor.o
 .bss.accly.3   0x0000000020000308        0x8 obj/mod_orientsensor.o
 .bss.acclz.1   0x0000000020000310        0x8 obj/mod_orientsensor.o
 .bss.events.0  0x0000000020000318        0x1 obj/mod_orientsensor.o
 *fill*         0x0000000020000319        0x7 
 .bss.pitch.8   0x0000000020000320        0x8 obj/mod_orientsensor.o
 .bss.ready.11  0x0000000020000328        0x1 obj/mod_orientsensor.o
 .bss.ready.12  0x0000000020000329        0x1 obj/mod_orientsensor.o
 *fill*         0x000000002000032a        0x6 
 .bss.roll.7    0x0000000020000330        0x8 obj/mod_orientsensor.o
 .bss.stored.10
                0x0000000020000338       0x18 obj/mod_orientsensor.o
 .bss.tiltx.6   0x0000000020000350        0x2 obj/mod_orientsensor.o
 .bss.tilty.4   0x0000000020000352        0x2 obj/mod_orientsensor.o
 .bss.tiltz.2   0x0000000020000354        0x2 obj/mod_orientsensor.o
 *fill*         0x0000000020000356        0x2 
 .bss.yaw.9     0x0000000020000358        0x8 obj/mod_orientsensor.o
 .bss.led_pos.3
                0x0000000020000360        0x1 obj/mod_lcd.o
 .bss.mode.0    0x0000000020000361        0x1 obj/mod_lighting.o
 *fill*         0x0000000020000362        0x2 
 .bss.task_prio.1
                0x0000000020000364        0x4 obj/mod_i2c.o
 .bss.pos.6     0x0000000020000368        0x1 obj/mod_radio.o
 *fill*         0x0000000020000369        0x3 
 .bss.radio_settings
                0x000000002000036c      0x140 obj/mod_radio.o
 .bss.rxd_config.5
                0x00000000200004ac        0x4 obj/mod_radio.o
 .bss.rxd_data.7
                0x00000000200004b0        0x4 obj/mod_radio.o
 .bss.state_info.0
                0x00000000200004b4       0x1a obj/mod_auxiliary.o
 *fill*         0x00000000200004ce        0x2 
 .bss.sys_data_mutex
                0x00000000200004d0        0x4 obj/mod_auxiliary.o
                0x00000000200004d0                sys_data_mutex
 .bss.system_info
                0x00000000200004d4       0x1a obj/mod_auxiliary.o
 *fill*         0x00000000200004ee        0x2 
 .bss.USB_Rx_ptr_out
                0x00000000200004f0        0x4 lib/libusb.a(hw_config.o)
                0x00000000200004f0                USB_Rx_ptr_out
 .bss.USB_Tx_Buffer
                0x00000000200004f4      0x800 lib/libusb.a(hw_config.o)
                0x00000000200004f4                USB_Tx_Buffer
 .bss.USB_Tx_State
                0x0000000020000cf4        0x1 lib/libusb.a(hw_config.o)
                0x0000000020000cf4                USB_Tx_State
 *fill*         0x0000000020000cf5        0x3 
 .bss.USB_Tx_length
                0x0000000020000cf8        0x4 lib/libusb.a(hw_config.o)
                0x0000000020000cf8                USB_Tx_length
 .bss.USB_Tx_ptr_in
                0x0000000020000cfc        0x4 lib/libusb.a(hw_config.o)
                0x0000000020000cfc                USB_Tx_ptr_in
 .bss.USB_Tx_ptr_out
                0x0000000020000d00        0x4 lib/libusb.a(hw_config.o)
                0x0000000020000d00                USB_Tx_ptr_out
 .bss.FrameCount.0
                0x0000000020000d04        0x4 lib/libusb.a(usb_endp.o)
 .bss.USB_Rx_Buffer
                0x0000000020000d08       0x40 lib/libusb.a(usb_endp.o)
                0x0000000020000d08                USB_Rx_Buffer
 .bss.USB_Rx_Counter
                0x0000000020000d48        0x2 lib/libusb.a(usb_endp.o)
                0x0000000020000d48                USB_Rx_Counter
 .bss.bIntPackSOF
                0x0000000020000d4a        0x1 lib/libusb.a(usb_istr.o)
                0x0000000020000d4a                bIntPackSOF
 *fill*         0x0000000020000d4b        0x1 
 .bss.wIstr     0x0000000020000d4c        0x2 lib/libusb.a(usb_istr.o)
                0x0000000020000d4c                wIstr
 .bss.Request   0x0000000020000d4e        0x1 lib/libusb.a(usb_prop.o)
                0x0000000020000d4e                Request
 *fill*         0x0000000020000d4f        0x1 
 .bss.bDeviceState
                0x0000000020000d50        0x4 lib/libusb.a(usb_pwr.o)
                0x0000000020000d50                bDeviceState
 .bss.Data_Mul_MaxPacketSize
                0x0000000020000d54        0x1 lib/libusb.a(usb_core.o)
                0x0000000020000d54                Data_Mul_MaxPacketSize
 *fill*         0x0000000020000d55        0x3 
 .bss.DevRemoteWakeup
                0x0000000020000d58        0x4 lib/libusb.a(usb_core.o)
                0x0000000020000d58                DevRemoteWakeup
 .bss.StatusInfo
                0x0000000020000d5c        0x2 lib/libusb.a(usb_core.o)
                0x0000000020000d5c                StatusInfo
 *fill*         0x0000000020000d5e        0x2 
 .bss.Device_Info
                0x0000000020000d60       0x1c lib/libusb.a(usb_init.o)
                0x0000000020000d60                Device_Info
 .bss.EPindex   0x0000000020000d7c        0x1 lib/libusb.a(usb_init.o)
                0x0000000020000d7c                EPindex
 *fill*         0x0000000020000d7d        0x3 
 .bss.pInformation
                0x0000000020000d80        0x4 lib/libusb.a(usb_init.o)
                0x0000000020000d80                pInformation
 .bss.pProperty
                0x0000000020000d84        0x4 lib/libusb.a(usb_init.o)
                0x0000000020000d84                pProperty
 .bss.pUser_Standard_Requests
                0x0000000020000d88        0x4 lib/libusb.a(usb_init.o)
                0x0000000020000d88                pUser_Standard_Requests
 .bss.wInterrupt_Mask
                0x0000000020000d8c        0x2 lib/libusb.a(usb_init.o)
                0x0000000020000d8c                wInterrupt_Mask
 .bss.SaveRState
                0x0000000020000d8e        0x2 lib/libusb.a(usb_int.o)
                0x0000000020000d8e                SaveRState
 .bss.SaveTState
                0x0000000020000d90        0x2 lib/libusb.a(usb_int.o)
                0x0000000020000d90                SaveTState
 *fill*         0x0000000020000d92        0x2 
 .bss.pxCurrentTCB
                0x0000000020000d94        0x4 lib/libfreertos.a(tasks.o)
                0x0000000020000d94                pxCurrentTCB
 .bss.pxDelayedTaskList
                0x0000000020000d98        0x4 lib/libfreertos.a(tasks.o)
 .bss.pxOverflowDelayedTaskList
                0x0000000020000d9c        0x4 lib/libfreertos.a(tasks.o)
 .bss.pxReadyTasksLists
                0x0000000020000da0       0x64 lib/libfreertos.a(tasks.o)
 .bss.uxCurrentNumberOfTasks
                0x0000000020000e04        0x4 lib/libfreertos.a(tasks.o)
 .bss.uxDeletedTasksWaitingCleanUp
                0x0000000020000e08        0x4 lib/libfreertos.a(tasks.o)
 .bss.uxPendedTicks
                0x0000000020000e0c        0x4 lib/libfreertos.a(tasks.o)
 .bss.uxSchedulerSuspended
                0x0000000020000e10        0x4 lib/libfreertos.a(tasks.o)
 .bss.uxTaskNumber
                0x0000000020000e14        0x4 lib/libfreertos.a(tasks.o)
 .bss.uxTopReadyPriority
                0x0000000020000e18        0x4 lib/libfreertos.a(tasks.o)
 .bss.xDelayedTaskList1
                0x0000000020000e1c       0x14 lib/libfreertos.a(tasks.o)
 .bss.xDelayedTaskList2
                0x0000000020000e30       0x14 lib/libfreertos.a(tasks.o)
 .bss.xIdleTaskHandle
                0x0000000020000e44        0x4 lib/libfreertos.a(tasks.o)
 .bss.xNextTaskUnblockTime
                0x0000000020000e48        0x4 lib/libfreertos.a(tasks.o)
 .bss.xNumOfOverflows
                0x0000000020000e4c        0x4 lib/libfreertos.a(tasks.o)
 .bss.xPendingReadyList
                0x0000000020000e50       0x14 lib/libfreertos.a(tasks.o)
 .bss.xSchedulerRunning
                0x0000000020000e64        0x4 lib/libfreertos.a(tasks.o)
 .bss.xSuspendedTaskList
                0x0000000020000e68       0x14 lib/libfreertos.a(tasks.o)
 .bss.xTasksWaitingTermination
                0x0000000020000e7c       0x14 lib/libfreertos.a(tasks.o)
 .bss.xTickCount
                0x0000000020000e90        0x4 lib/libfreertos.a(tasks.o)
 .bss.xYieldPending
                0x0000000020000e94        0x4 lib/libfreertos.a(tasks.o)
 .bss.pxCurrentTimerList
                0x0000000020000e98        0x4 lib/libfreertos.a(timers.o)
 .bss.pxOverflowTimerList
                0x0000000020000e9c        0x4 lib/libfreertos.a(timers.o)
 .bss.xActiveTimerList1
                0x0000000020000ea0       0x14 lib/libfreertos.a(timers.o)
 .bss.xActiveTimerList2
                0x0000000020000eb4       0x14 lib/libfreertos.a(timers.o)
 .bss.xLastTime.0
                0x0000000020000ec8        0x4 lib/libfreertos.a(timers.o)
 .bss.xTimerQueue
                0x0000000020000ecc        0x4 lib/libfreertos.a(timers.o)
 .bss.xTimerTaskHandle
                0x0000000020000ed0        0x4 lib/libfreertos.a(timers.o)
 .bss.pxEnd     0x0000000020000ed4        0x4 lib/libfreertos.a(heap_4.o)
 .bss.ucHeap    0x0000000020000ed8     0x3c00 lib/libfreertos.a(heap_4.o)
                0x0000000020000ed8                ucHeap
 .bss.xBlockAllocatedBit
                0x0000000020004ad8        0x4 lib/libfreertos.a(heap_4.o)
 .bss.xFreeBytesRemaining
                0x0000000020004adc        0x4 lib/libfreertos.a(heap_4.o)
 .bss.xMinimumEverFreeBytesRemaining
                0x0000000020004ae0        0x4 lib/libfreertos.a(heap_4.o)
 .bss.xStart    0x0000000020004ae4        0x8 lib/libfreertos.a(heap_4.o)
 .bss.__lock___sfp_recursive_mutex
                0x0000000020004aec        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
                0x0000000020004aec                __lock___sfp_recursive_mutex
 .bss.__lock___sinit_recursive_mutex
                0x0000000020004aed        0x1 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
                0x0000000020004aed                __lock___sinit_recursive_mutex
 *fill*         0x0000000020004aee        0x2 
 .bss.errno     0x0000000020004af0        0x4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
                0x0000000020004af0                errno
 *(COMMON)
                0x0000000020004af4                . = ALIGN (0x4)
                0x0000000020004af4                _ebss = .
                0x0000000020004af4                __bss_end__ = _ebss
                [!provide]                        PROVIDE (end = _ebss)
                [!provide]                        PROVIDE (_end = _ebss)
                0x0000000020005000                __StackTop = (ORIGIN (RAM) + LENGTH (RAM))
                0x0000000020004e00                __StackLimit = (__StackTop - STACK_SIZE)
                0x0000000020005000                _estack = __StackTop

.heap           0x0000000020004af4        0x0
                0x0000000020004af4                . = ALIGN (0x4)
                0x0000000020004af4                __HeapBase = .
                0x0000000020004af4                . = (. + HEAP_SIZE)
                0x0000000020004af4                . = ALIGN (0x4)
                0x0000000020004af4                __HeapLimit = .

.stack          0x0000000020004e00      0x200 load address 0x0000000008015948
                0x0000000020004e00                . = ALIGN (0x4)
                0x0000000020005000                . = (. + STACK_SIZE)
 *fill*         0x0000000020004e00      0x200 
                0x0000000020005000                . = ALIGN (0x4)
                0x0000000020005000                __stack = .

.memory_b1_text
 *(.mb1text)
 *(.mb1text*)
 *(.mb1rodata)
 *(.mb1rodata*)

/DISCARD/
 libc.a(*)
 libm.a(*)
 libgcc.a(*)
OUTPUT(obj/project.elf elf32-littlearm)
LOAD linker stubs
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc.a
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a
LOAD /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a

.comment        0x0000000000000000       0x4d
 .comment       0x0000000000000000       0x4d obj/mod_led.o
                                         0x4e (size before relaxing)
 .comment       0x000000000000004d       0x4e obj/mod_sound.o
 .comment       0x000000000000004d       0x4e obj/mod_sensors.o
 .comment       0x000000000000004d       0x4e obj/mod_drive.o
 .comment       0x000000000000004d       0x4e obj/mod_orientsensor.o
 .comment       0x000000000000004d       0x4e obj/mod_lcd.o
 .comment       0x000000000000004d       0x4e obj/mod_lighting.o
 .comment       0x000000000000004d       0x4e obj/mod_console.o
 .comment       0x000000000000004d       0x4e obj/mod_i2c.o
 .comment       0x000000000000004d       0x4e obj/mod_radio.o
 .comment       0x000000000000004d       0x4e obj/mod_auxiliary.o
 .comment       0x000000000000004d       0x4e /tmp/ccPh1D7g.o
 .comment       0x000000000000004d       0x4e /tmp/cchESTC9.o
 .comment       0x000000000000004d       0x4e /tmp/cc9ErQe2.o
 .comment       0x000000000000004d       0x4e /tmp/cc9a8G1U.o
 .comment       0x000000000000004d       0x4e /tmp/ccTyJAXN.o
 .comment       0x000000000000004d       0x4e /tmp/ccrbnG3G.o
 .comment       0x000000000000004d       0x4e lib/libusb.a(hw_config.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_desc.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_endp.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_istr.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_prop.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_pwr.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_core.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_init.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_int.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_mem.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_regs.o)
 .comment       0x000000000000004d       0x4e lib/libusb.a(usb_sil.o)
 .comment       0x000000000000004d       0x4e lib/libfreertos.a(tasks.o)
 .comment       0x000000000000004d       0x4e lib/libfreertos.a(list.o)
 .comment       0x000000000000004d       0x4e lib/libfreertos.a(queue.o)
 .comment       0x000000000000004d       0x4e lib/libfreertos.a(timers.o)
 .comment       0x000000000000004d       0x4e lib/libfreertos.a(port.o)
 .comment       0x000000000000004d       0x4e lib/libfreertos.a(heap_4.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(misc.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_rcc.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_gpio.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_exti.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_tim.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_adc.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_usart.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_pwr.o)
 .comment       0x000000000000004d       0x4e lib/libstm32.a(stm32f10x_dma.o)

.debug_frame    0x0000000000000000     0x1890
 .debug_frame   0x0000000000000000       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-w_atan2.o)
 .debug_frame   0x0000000000000020       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-e_atan2.o)
 .debug_frame   0x0000000000000068       0x38 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_atan.o)
 .debug_frame   0x00000000000000a0       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libm.a(lib_a-s_fabs.o)
 .debug_frame   0x00000000000000c0       0xac /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubdf3.o)
 .debug_frame   0x000000000000016c       0x50 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivdf3.o)
 .debug_frame   0x00000000000001bc       0xc4 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpdf2.o)
 .debug_frame   0x0000000000000280       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixunsdfsi.o)
 .debug_frame   0x00000000000002a4       0x4c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_addsubsf3.o)
 .debug_frame   0x00000000000002f0       0x38 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_muldivsf3.o)
 .debug_frame   0x0000000000000328       0xc8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_cmpsf2.o)
 .debug_frame   0x00000000000003f0       0x30 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atoi.o)
 .debug_frame   0x0000000000000420       0x30 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-atol.o)
 .debug_frame   0x0000000000000450       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-errno.o)
 .debug_frame   0x0000000000000470       0x5c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fflush.o)
 .debug_frame   0x00000000000004cc       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fileno.o)
 .debug_frame   0x00000000000004f8      0x14c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-findfp.o)
 .debug_frame   0x0000000000000644       0x54 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fwalk.o)
 .debug_frame   0x0000000000000698       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getchar.o)
 .debug_frame   0x00000000000006e0       0xb0 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lock.o)
 .debug_frame   0x0000000000000790       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memcpy-stub.o)
 .debug_frame   0x00000000000007b8       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memset.o)
 .debug_frame   0x00000000000007d8       0x94 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_float.o)
 .debug_frame   0x000000000000086c       0x60 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf_i.o)
 .debug_frame   0x00000000000008cc       0x74 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-printf.o)
 .debug_frame   0x0000000000000940       0x5c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-reent.o)
 .debug_frame   0x000000000000099c       0x74 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-scanf.o)
 .debug_frame   0x0000000000000a10       0x38 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-setvbuf.o)
 .debug_frame   0x0000000000000a48       0x7c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-snprintf.o)
 .debug_frame   0x0000000000000ac4       0x88 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-stdio.o)
 .debug_frame   0x0000000000000b4c       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strcat.o)
 .debug_frame   0x0000000000000b74       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strchr.o)
 .debug_frame   0x0000000000000b94       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncmp.o)
 .debug_frame   0x0000000000000bbc       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strncpy.o)
 .debug_frame   0x0000000000000be4       0x64 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtol.o)
 .debug_frame   0x0000000000000c48       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-syswrite.o)
 .debug_frame   0x0000000000000c68       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-writer.o)
 .debug_frame   0x0000000000000c94       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-closer.o)
 .debug_frame   0x0000000000000cc0       0x6c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-dtoa.o)
 .debug_frame   0x0000000000000d2c       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-getc.o)
 .debug_frame   0x0000000000000d74       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-localeconv.o)
 .debug_frame   0x0000000000000db4       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-lseekr.o)
 .debug_frame   0x0000000000000de0       0x58 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-makebuf.o)
 .debug_frame   0x0000000000000e38       0x30 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-malloc.o)
 .debug_frame   0x0000000000000e68       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memchr-stub.o)
 .debug_frame   0x0000000000000e90      0x254 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mprec.o)
 .debug_frame   0x00000000000010e4       0x90 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-svfprintf.o)
 .debug_frame   0x0000000000001174       0xac /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfprintf.o)
 .debug_frame   0x0000000000001220       0xa8 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf.o)
 .debug_frame   0x00000000000012c8       0x64 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-nano-vfscanf_i.o)
 .debug_frame   0x000000000000132c       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-readr.o)
 .debug_frame   0x0000000000001358       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-refill.o)
 .debug_frame   0x0000000000001398       0x3c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-rget.o)
 .debug_frame   0x00000000000013d4       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-sccl.o)
 .debug_frame   0x0000000000001400       0x64 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-strtoul.o)
 .debug_frame   0x0000000000001464       0x5c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-ungetc.o)
 .debug_frame   0x00000000000014c0       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wbuf.o)
 .debug_frame   0x0000000000001500       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wsetup.o)
 .debug_frame   0x000000000000152c       0x40 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-assert.o)
 .debug_frame   0x000000000000156c       0x64 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fprintf.o)
 .debug_frame   0x00000000000015d0       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-fstatr.o)
 .debug_frame   0x00000000000015fc       0x2c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-isattyr.o)
 .debug_frame   0x0000000000001628       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-locale.o)
 .debug_frame   0x0000000000001670       0x48 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-mbtowc_r.o)
 .debug_frame   0x00000000000016b8       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-memmove.o)
 .debug_frame   0x00000000000016e0       0x3c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-wctomb_r.o)
 .debug_frame   0x000000000000171c       0x28 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-abort.o)
 .debug_frame   0x0000000000001744       0xcc /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signal.o)
 .debug_frame   0x0000000000001810       0x3c /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/../../../../arm-none-eabi/lib/thumb/v7-m/nofp/libc_nano.a(lib_a-signalr.o)
 .debug_frame   0x000000000000184c       0x20 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_unorddf2.o)
 .debug_frame   0x000000000000186c       0x24 /home/lukasz/Desktop/work/AMU/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/../lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/libgcc.a(_arm_fixdfsi.o)
