/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_ORIENT_SENSOR_H
#define MOD_ORIENT_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define SEN_ACCL_VALS_SIZE      900U

typedef struct {
    double x;
    double y;
    double z;
} Vector_t;

typedef struct {
    Vector_t vect;
    uint8_t event;
} AcclData_t;

typedef enum {
    ACCL_EVENT_NONE = 0,
    ACCL_EVENT_TAP,
} AcclEvent_t;

void vOrientSensorServiceTask(void * pvArg);
void gyro_get_data(Vector_t *data);
void accl_get_data(AcclData_t *data);

extern bool is_robot_stopped(void);

#endif
