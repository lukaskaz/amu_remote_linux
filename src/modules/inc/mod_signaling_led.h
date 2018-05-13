/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_SIGNALING_LED_H
#define MOD_SIGNALING_LED_H

#include "FreeRTOS.h"
#include "semphr.h"

extern xSemaphoreHandle xSemaphLedSignal;
    
void vLED_Configuration(void);
void vLedSignalingTask(void * pvArg);


#endif
