/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_SENSORS_H
#define MOD_SENSORS_H

#include <stdint.h>
#include "stm32f10x.h"

typedef enum {
    SENSOR_FRONT = 0,
    SENSOR_REAR,
} sensorType_t;

typedef enum {
    SEN_BAT_LOW = 0,
    SEN_BAT_HALF,
    SEN_BAT_FULL
} sensorBattery_t;

typedef enum {
    SEN_PROX_NONE = 0,
    SEN_PROX_WRN_1,
    SEN_PROX_WRN_2,
    SEN_PROX_ALERT,
} sensorProximity_t;

extern void vSensorsServiceTask(void * pvArg);
extern void vCheckSupplyVoltage(FlagStatus PVDO);
extern sensorBattery_t get_battery_status(void);
extern void adjust_auto_lighting(void);
extern sensorProximity_t get_front_proximity_estimation(void);

extern double distance[];
extern sensorType_t sensorInUse;

#endif
