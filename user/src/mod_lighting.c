/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_lighting.c
** Descriptions:            Interface for lighting control
**
**--------------------------------------------------------------------------------------------------------
** Created by:              
** Created date:            
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "mod_lcd.h"
#include "mod_sensors.h"
#include "mod_lighting.h"

typedef enum {
    LIGHT_BRIGHT_NONE = 10,
    LIGHT_BRIGHT_MIN  = 20,
    LIGHT_BRIGHT_MID  = 45,
    LIGHT_BRIGHT_MAX  = 70,
    LIGHT_BRIGHT_HIST = 2,
} lightBright_t;

typedef enum {
    LIGHT_AUTO_UNINIT = 0,
    LIGHT_AUTO_NONE,
    LIGHT_AUTO_MIN,
    LIGHT_AUTO_MEDIUM,
    LIGHT_AUTO_MAX
} LightAutoMode_t;


static lightMode_t sLighting_set_mode(lightMode_t*);
static lightMode_t sLighting_get_mode(void);
static void vLighting_Auto_Control(LightAutoMode_t);


bool bLighting_is_auto(void)
{
    return sLighting_get_mode() == LIGHT_MODE_AUTO ? true:false;
}

void vLighting_set_auto(bool is_auto)
{
    lightMode_t mode = true == is_auto ? LIGHT_MODE_AUTO:LIGHT_MODE_MANUAL;
    sLighting_set_mode(&mode);
}

lightMode_t sLighting_set_mode(lightMode_t* mode)
{
    static lightMode_t light_mode = LIGHT_MODE_AUTO;
    if(NULL != mode) light_mode = *mode;
    return light_mode;
}

lightMode_t sLighting_get_mode(void)
{
    return sLighting_set_mode(NULL);
}

void vLighting_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);

    // SWJ Disabled (JTAG-DP + SW-DP) and I/O Pins enabled instead
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

#define LIGHT_GET_LED(data)         GPIO_ReadOutputDataBit(data.port, data.pin)
#define LIGHT_SET_LED(data,state)   GPIO_WriteBit(data.port, data.pin, state)
#define LIGHT_TOGGLE_LED(data)      LIGHT_SET_LED(data, !LIGHT_GET_LED(data))

void vLighting_Control(uint8_t selected_led)
{
    enum light_t { LIGHT_FIRST = 0, LIGHT_NONE = LIGHT_FIRST, LIGHT_LEFT,
            LIGHT_RIGHT, LIGHT_INNER, LIGHT_OUTER, LIGHT_LAST };
    static const struct {
        GPIO_TypeDef * port;
        uint16_t pin;
    } lighting_leds[LIGHT_LAST] = {
        [LIGHT_LEFT]  = { GPIOB, GPIO_Pin_5 }, [LIGHT_RIGHT] = { GPIOB, GPIO_Pin_4  },
        [LIGHT_INNER] = { GPIOB, GPIO_Pin_3 }, [LIGHT_OUTER] = { GPIOA, GPIO_Pin_15 },
    };

    if(LIGHT_FIRST < selected_led && LIGHT_LAST > selected_led)
        LIGHT_TOGGLE_LED(lighting_leds[selected_led]);
    if(LIGHT_NONE == selected_led)
        for(enum light_t led = LIGHT_FIRST+1; led < LIGHT_LAST; led++)
            LIGHT_SET_LED(lighting_leds[led], Bit_SET);
}

void vLighting_set_state(const light_settings_t* __restrict light)
{
    vLighting_RF_Control(LIGHT_RF_LEFT,  light->left);
    vLighting_RF_Control(LIGHT_RF_RIGHT, light->right);
    vLighting_RF_Control(LIGHT_RF_INNER, light->inner);
    vLighting_RF_Control(LIGHT_RF_OUTER, light->outer);
}

light_settings_t vLighting_get_state(void)
{
    light_settings_t light = {0};
    light.autodetect = bLighting_is_auto();
    light.all = (light.all & 0xF0) | vGet_Lighting_State();
    return light;
}

uint8_t vGet_Lighting_State(void)
{
    uint8_t state = LIGHT_RF_NONE;

    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5) == Bit_RESET) {
        state |= LIGHT_RF_LEFT;
    }
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET) {
        state |= LIGHT_RF_RIGHT;
    }
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) == Bit_RESET) {
        state |= LIGHT_RF_INNER;
    }
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET) {
        state |= LIGHT_RF_OUTER;
    }

    return state;
}

void vLighting_RF_Control(const uint8_t type, const uint8_t state)
{
    BitAction action = LIGHT_RF_ENABLE == state ? Bit_RESET:Bit_SET;

    switch(type) {
        case LIGHT_RF_NONE:
            GPIO_SetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            break;
        case LIGHT_RF_LEFT: GPIO_WriteBit(GPIOB, GPIO_Pin_5, action); break;
        case LIGHT_RF_RIGHT: GPIO_WriteBit(GPIOB, GPIO_Pin_4, action); break;
        case LIGHT_RF_INNER: GPIO_WriteBit(GPIOB, GPIO_Pin_3, action); break;
        case LIGHT_RF_OUTER: GPIO_WriteBit(GPIOA, GPIO_Pin_15, action); break;
        default: printf("Lighting operation not supported!\n\r"); break;
    }
}

void vLighting_Auto_Control(LightAutoMode_t type)
{
    switch(type) {
        case LIGHT_AUTO_NONE:
            GPIO_SetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            break;
        case LIGHT_AUTO_MIN:
            GPIO_ResetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_4);
            GPIO_SetBits(GPIOB, GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            break;
        case LIGHT_AUTO_MEDIUM:
            GPIO_ResetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            break;
        case LIGHT_AUTO_MAX:
            GPIO_ResetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            break;
        default: {}
    }
}

void vLighting_adjust_auto_light(void)
{
    static LightAutoMode_t mode = LIGHT_AUTO_UNINIT;
    const double bright = get_illumination();

    if((LIGHT_BRIGHT_MAX + LIGHT_BRIGHT_HIST) < bright) {
        mode = LIGHT_AUTO_NONE;
    }
    else if((LIGHT_BRIGHT_MAX - LIGHT_BRIGHT_HIST) > bright &&
            (LIGHT_BRIGHT_MID + LIGHT_BRIGHT_HIST) < bright) {
        mode = LIGHT_AUTO_MIN;
    }
    else if((LIGHT_BRIGHT_MID - LIGHT_BRIGHT_HIST) > bright &&
            (LIGHT_BRIGHT_MIN + LIGHT_BRIGHT_HIST) < bright) {
        mode = LIGHT_AUTO_MEDIUM;
    }
    else if((LIGHT_BRIGHT_MIN - LIGHT_BRIGHT_HIST) > bright) {
        mode = LIGHT_AUTO_MAX;
    }
    else {
        if(LIGHT_AUTO_UNINIT == mode) {
            mode = bright > LIGHT_BRIGHT_MAX ? LIGHT_AUTO_NONE: \
                   bright > LIGHT_BRIGHT_MID ? LIGHT_AUTO_MIN:  \
                   bright > LIGHT_BRIGHT_MIN ? LIGHT_AUTO_MEDIUM:LIGHT_AUTO_MAX;
        }
    }
    vLighting_Auto_Control(mode);
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
