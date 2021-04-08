#ifndef __MOD_ORIENT_SENSOR_H__
#define __MOD_ORIENT_SENSOR_H__

#include <stdint.h>
#include <stdbool.h>

#define SEN_ACCL_VALS_SIZE      900U

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} VectorInt_t;

typedef struct {
    double x;
    double y;
    double z;
} VectorDbl_t;

typedef struct {
    VectorDbl_t vect;
    uint8_t event;
} AcclData_t;

typedef enum {
    ACCL_EVENT_NONE = 0,
    ACCL_EVENT_TAP,
} AcclEvent_t;

//extern void vOrientSensorServiceTask(void * pvArg);
extern void vOrientation_sensor_configuration(void);
void gyro_get_data(TickType_t);
void accl_get_data(TickType_t);
extern double sensor_get_acclx(void);
extern double sensor_get_accly(void);
extern double sensor_get_acclz(void);
extern uint16_t sensor_get_tiltx(void);
extern uint16_t sensor_get_tilty(void);
extern uint16_t sensor_get_tiltz(void);
extern uint16_t sensor_get_yaw(void);
extern uint16_t sensor_get_pitch(void);
extern uint16_t sensor_get_roll(void);

extern bool is_robot_stopped(void);

#endif  // __MOD_ORIENT_SENSOR_H__
