/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_SOUND_SIGNAL_H
#define MOD_SOUND_SIGNAL_H

#include "stm32f10x.h"

typedef enum {
    SOUND_RF_NONE = 0,
    SOUND_RF_PLAIN,
} SoundRFOperation_t;

void vSound_configuration(void);
void vSound_Signal_Console(void);
void vSound_Signal_RF_Control(const uint8_t status);

#endif
