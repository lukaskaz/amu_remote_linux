/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_LCD_H
#define MOD_LCD_H

#include "FreeRTOS.h"
#include "queue.h"

#include "stm32f10x.h"


typedef enum {
    LCD_OP_NONE = 0,
    LCD_OP_DRIVE,
    LCD_OP_LIGHTING,
    LCD_OP_SOUND_SIG,
    LCD_OP_COLLISION,
} LcdOperation_t;

typedef enum {
    LCD_SOUND_OFF = 0,
    LCD_SOUND_ON,
} LcdSoundState_t;

typedef enum {
    LCD_LIGHT_MANUAL = 0,
    LCD_LIGHT_AUTO,
} LcdLightState_t;

typedef enum {
    LCD_MV_STOPPED = 0,
    LCD_MV_IN_MOTION,
} LcdMovementState_t;

typedef enum {
    LCD_COLLIS_OFF = 0,
    LCD_COLLIS_ON,
} LcdCollision_t;

typedef struct {
    uint8_t operation;
    uint8_t state;
    uint8_t value;
} lcdControlData_t;


void vLcdInterfaceTask(void * pvArg);

extern xQueueHandle xQueueLcdControl;

#endif
