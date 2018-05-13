/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_LIGHTING_H
#define MOD_LIGHTING_H

#include "stm32f10x.h"
#include <stdbool.h>


typedef enum {
    LIGHT_AUTO_NONE = 0,
    LIGHT_AUTO_MIN,
    LIGHT_AUTO_MEDIUM,
    LIGHT_AUTO_MAX
} LightAutoOperation_t;

extern bool isLightingAutoModeEnabled(void);
extern void vLighting_configuration(void);
extern void vLighting_Console(void);
extern void vLighting_RF_Control(const uint8_t type, const uint8_t state);
extern void vLighting_Auto_Control(const LightAutoOperation_t type);

void vGet_Lighting_State(uint8_t* state);
#endif
