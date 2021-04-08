#ifndef __MOD_LIGHTING_H__
#define __MOD_LIGHTING_H__

#include <stdbool.h>
#include "mod_auxiliary.h"

#define LIGHT_LED_BIT_POS_MIN   1
#define LIGHT_LED_BIT_POS_MAX   4

typedef enum {
    LIGHT_RF_FIRST = 0x00,
    LIGHT_RF_NONE  = LIGHT_RF_FIRST,
    LIGHT_RF_LEFT  = 0x01,
    LIGHT_RF_RIGHT = 0x02,
    LIGHT_RF_INNER = 0x04,
    LIGHT_RF_OUTER = 0x08,
    LIGHT_RF_LAST  = 0x10
} LightRFOperation_t;


typedef enum {
    LIGHT_RF_DISABLE = 0,
    LIGHT_RF_ENABLE,
} LightState_t;

typedef enum {
    LIGHT_MODE_MANUAL = 0,
    LIGHT_MODE_AUTO,
} lightMode_t;


extern bool bLighting_is_auto(void);
extern void vLighting_set_auto(bool);
extern void vLighting_Control(uint8_t);
extern void vLighting_configuration(void);
extern void vLighting_RF_Control(const uint8_t type, const uint8_t state);

void vLighting_set_state(const light_settings_t* __restrict);
extern light_settings_t vLighting_get_state(void);

extern uint8_t vGet_Lighting_State(void);
extern void vLighting_adjust_auto_light(void);

#endif  // __MOD_LIGHTING_H__
