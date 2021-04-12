#ifndef __MOD_DRIVE_H__
#define __MOD_DRIVE_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "mod_auxiliary.h"

#define DRIVE_SPEED_MAX             100

typedef enum {
    DRIVE_OP_STOPPED = 0,
    DRIVE_OP_FORWARD,
    DRIVE_OP_BACKWARD,
    DRIVE_OP_LEFT,
    DRIVE_OP_RIGHT,
    DRIVE_OP_JOY_STOPPED,
    DRIVE_OP_JOY_FORWARD,
    DRIVE_OP_JOY_BACKWARD,
    DRIVE_OP_JOY_FW_LEFT,
    DRIVE_OP_JOY_FW_RIGHT,
    DRIVE_OP_JOY_BW_LEFT,
    DRIVE_OP_JOY_BW_RIGHT,
    DRIVE_OP_WAIT,
} driveOperations_t;

typedef struct {
    uint8_t controller;
    uint8_t direction;
    uint8_t speed_0;
    uint8_t speed_1;
} driveControlData_t;

extern void vDrive_Configuration(void);
extern void vDriveTask(void *pvArg);


#endif  // __MOD_DRIVE_H__
