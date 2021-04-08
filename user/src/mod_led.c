/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_signaling_led.c
** Descriptions:            Interface for controller's signaling blue led
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

#include "mod_led.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mod_drive.h"

typedef enum {
    LED_BRIGHTEN = 0,
    LED_FADING,
} ledLighting_t;

#define LED_TASK_INTERVAL_MS    50
#define LED_PWM_TIMER_STEPS     50
#define LED_PWM_TIMER_FREQ      500
#define LED_BRIGHT_MAX          LED_PWM_TIMER_STEPS
#define LED_BRIGHT_STEP         3
#define LED_BRIGHT_MIN          0
#define LED_SET_BRIGHT_DIR(dir,val) \
    LED_BRIGHTEN == dir && LED_BRIGHT_MAX <= val ? LED_FADING: \
    LED_FADING == dir && LED_BRIGHT_MIN >= val ? LED_BRIGHTEN:dir
#define LED_CALC_BRIGHT_LVL(dir,val) \
    LED_BRIGHTEN == dir ? val+LED_BRIGHT_STEP:val-LED_BRIGHT_STEP;

void led_update_pwm_duty(void)
{
    led_set_brightness_level(NULL);
}

void led_set_brightness_level(int16_t* lvl)
{
    static volatile uint16_t current_lvl = 0;

    if(NULL != lvl) {
        *lvl = LED_BRIGHT_MAX >= *lvl ? *lvl:LED_BRIGHT_MAX;
        *lvl = LED_BRIGHT_MIN <= *lvl ? *lvl:LED_BRIGHT_MIN;
        current_lvl = *lvl;
        TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
    }
    else TIM_SetCompare3(TIM1, current_lvl);
}

/*******************************************************************************
* Function Name  : vLED_Configuration
* Description    : Configure GPIO Pin for signaling LED
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void vLED_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    RCC_ClocksTypeDef RCC_Clocks = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOB, ENABLE);

    // Bind function TIM1 CH3N to pin PB1
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_GetClocksFreq(&RCC_Clocks);
    TIM_TimeBaseStructure.TIM_Period = \
        TIMER_STEPS_TO_PERIOD(LED_PWM_TIMER_STEPS);
    TIM_TimeBaseStructure.TIM_Prescaler = \
        TIMER_FREQHZ_TO_PRESCAL(RCC_Clocks, LED_PWM_TIMER_STEPS, LED_PWM_TIMER_FREQ);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/*******************************************************************************
* Function Name  : vLedSignalingTask
* Description    : LED signaling Task
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void vLedTask(void *pvArg)
{
    ledLighting_t ledLightDir = LED_BRIGHTEN;
    int16_t ledBrightVal  = 0;
    const TickType_t interval = pdMS_TO_TICKS(LED_TASK_INTERVAL_MS);

    vLED_Configuration();
    while(1) {
        ledLightDir = LED_SET_BRIGHT_DIR(ledLightDir, ledBrightVal);
        ledBrightVal = LED_CALC_BRIGHT_LVL(ledLightDir, ledBrightVal);
        led_set_brightness_level(&ledBrightVal);
        vTaskDelay(interval);
    }

}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
