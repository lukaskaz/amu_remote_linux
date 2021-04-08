#ifndef __MOD_SIGNALING_LED_H__
#define __MOD_SIGNALING_LED_H__

#include "FreeRTOS.h"
#include "semphr.h"

extern void vLED_Configuration(void);
extern void vLedTask(void * pvArg);
extern void led_set_brightness_level(int16_t*);
extern void led_update_pwm_duty(void);

#endif  // __MOD_SIGNALING_LED_H__
