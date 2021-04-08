#ifndef __MOD_AUXILIARY_H__
#define __MOD_AUXILIARY_H__

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

#define SIZEOF_ARRAY(arr)        sizeof(arr)/sizeof(arr[0])

#define TIMER_STEPS_TO_PERIOD(steps)  (steps - 1)
#define TIMER_FREQHZ_TO_PRESCAL(rcc, steps, freq) \
                ((rcc.HCLK_Frequency/(freq*steps)) - 1)

#define UNIQUE_NAME(name,tag)    name ##_## tag
#define DUMMY_NAME(name,tag)     UNIQUE_NAME(name,tag)
#define DUMMY_MEMBER             DUMMY_NAME(reserved, __LINE__)

typedef enum {
    DATA_GET_ONLY = 0,
    DATA_LOCK_AND_GET,
    DATA_SET_AND_UNLOCK
} state_data_op_t;

typedef struct {
    uint8_t direction;
    uint8_t speed_lft;
    uint8_t speed_rgt;
} movement_settings_t;

typedef union {
    uint8_t all;
    struct {
        uint8_t left:1;
        uint8_t right:1;
        uint8_t inner:1;
        uint8_t outer:1;
        uint8_t autodetect:1;
        uint8_t DUMMY_MEMBER:3;
    };
} light_settings_t;

typedef struct {
    uint8_t honk:2;
    uint8_t DUMMY_MEMBER:6;
} signal_settings_t;

typedef struct {
    movement_settings_t movement;
    light_settings_t light;
    signal_settings_t signal;
} exec_settings_t;

typedef struct {
    uint8_t rollex:2;
    uint8_t pitchex:2;
    uint8_t yawex:2;
    uint8_t DUMMY_MEMBER:2;
    uint8_t acclxex:2;
    uint8_t acclyex:2;
    uint8_t acclzex:2;
    uint8_t DUMMY_MEMBER:2;
    uint8_t collision;
    uint8_t roll;
    uint8_t pitch;
    uint8_t yaw;
    uint8_t acclx;
    uint8_t accly;
    uint8_t acclz;
} orientation_data_t;

typedef struct {
    uint8_t tempex:2;
    uint8_t frontdistex:2;
    uint8_t backdistex:2;
    uint8_t DUMMY_MEMBER:2;
    uint8_t light;
    uint8_t temp;
    uint8_t battery;
    uint8_t frontdist;
    uint8_t backdist;
} enviro_data_t;

typedef struct __attribute__((packed)) {
    uint8_t start_byte;
    exec_settings_t exec;
    uint8_t guard_first;
    orientation_data_t orientation;
    uint8_t guard_second;
    enviro_data_t enviro;
    uint8_t chksum;
    uint8_t end_byte;
} radio_frame_t;

typedef struct {
    bool radio_comm_active;
    radio_frame_t radio_frame_send;
} system_state_t;

#define ARR

extern SemaphoreHandle_t sys_data_mutex;

extern uint8_t auxil_get_bit_from_pos(uint8_t);

extern radio_frame_t* radio_get_frame_recv(void);
extern ErrorStatus radio_get_status_curr(radio_frame_t* __restrict);
extern ErrorStatus radio_set_status_curr(radio_frame_t* __restrict);
extern ErrorStatus exec_group_set_status(const exec_settings_t* __restrict);
extern exec_settings_t exec_group_get_status(void);
extern movement_settings_t drive_get_status(void);
extern uint8_t drive_get_direction(void);
extern ErrorStatus drive_set_status(const movement_settings_t* __restrict);
extern light_settings_t light_get_status(void);
extern ErrorStatus light_set_status(const light_settings_t*);
extern bool light_is_auto(void);
extern ErrorStatus light_set_auto(bool);
extern ErrorStatus light_toggle_led(uint8_t);
extern ErrorStatus light_set_led_on(uint8_t);
extern ErrorStatus light_off_all(void);
extern uint8_t light_get_leds_on(void);
extern bool signal_is_on(void);
extern ErrorStatus signal_set_state(uint8_t);
extern uint8_t signal_get_state(void);

#endif  // __MOD_AUXILIARY_H__
