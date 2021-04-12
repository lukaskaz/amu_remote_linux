#ifndef __MOD_SENSORS_H__
#define __MOD_SENSORS_H__

#include <stdint.h>
#include "stm32f10x.h"

typedef enum {
    SENSOR_DIST_FIRST = 0,
    SENSOR_DIST_FRONT = SENSOR_DIST_FIRST,
    SENSOR_DIST_REAR,
    SENSOR_DIST_LAST,
} sensorDistance_t;

typedef enum {
    SEN_BAT_LOW = 0,
    SEN_BAT_HALF,
    SEN_BAT_FULL
} sensorBattery_t;

typedef enum {
    SEN_PROX_NONE = 0,
    SEN_PROX_WARN,
    SEN_PROX_ALERT,
} sensorProximity_t;

typedef enum {
    SEN_VOLT_2V2 = 0,
    SEN_VOLT_2V3,
    SEN_VOLT_2V4,
    SEN_VOLT_2V5,
    SEN_VOLT_2V6,
    SEN_VOLT_2V7,
    SEN_VOLT_2V8,
    SEN_VOLT_2V9
} sensorVolatage_t;

extern void vSensorsTask(void * pvArg);
extern void vCheckSupplyVoltage(FlagStatus PVDO);
extern sensorBattery_t get_battery_status(void);
extern sensorProximity_t get_front_proximity_estimation(void);

uint8_t PWR_PVDLevelGet(void);
double get_illumination(void);
double get_internal_temp(void);
uint16_t get_front_distance(void);
uint16_t get_back_distance(void);

extern double distance[];
extern sensorDistance_t sensorInUse;

#endif  // __MOD_SENSORS_H__
