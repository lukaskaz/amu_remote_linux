/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_DRIVE_H
#define MOD_DRIVE_H

#include "FreeRTOS.h"
#include "queue.h"

#include "stm32f10x.h"


typedef enum {
    DRIVE_OP_STOPPED = 0,
    DRIVE_OP_FORWARD,
    DRIVE_OP_BACKWARD,
    DRIVE_OP_LEFT,
    DRIVE_OP_RIGHT,
    DRIVE_OP_JOY_STOPPED,
    DRIVE_OP_JOY_FORWARD,
    DRIVE_OP_JOY_BACKWARD,
    DRIVE_OP_JOY_LEFT,
    DRIVE_OP_JOY_RIGHT,
} driveOperations_t;

typedef struct {
    uint8_t controller;
    uint8_t direction;
    uint8_t speed_0;
    uint8_t speed_1;
} driveControlData_t;


void vDrive_Console(void);
void vDrive_Control(void *pvArg);


extern xQueueHandle xQueueDriveControlCmd;
#endif
