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

#include "mod_signaling_led.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

typedef enum {
    LED_BRIGHTEN = 0,
    LED_FADING,
} ledLighting_t ;

xSemaphoreHandle xSemaphLedSignal = NULL;

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
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
    RCC_ClocksTypeDef        RCC_Clocks;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;   //for PWM control
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //for bistate control
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //return;

    RCC_GetClocksFreq(&RCC_Clocks);
    TIM_TimeBaseStructure.TIM_Period        = 100U - 1U;
    TIM_TimeBaseStructure.TIM_Prescaler     = RCC_Clocks.HCLK_Frequency/(500U*100U) - 1U;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    /* TIM_SIG_PWM_TIMER enable counter */ 
    TIM_Cmd(TIM3, ENABLE);
    TIM3->CCR4 = 0;
}

/*******************************************************************************
* Function Name  : vLedSignalingTask
* Description    : LED signaling Task
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
#define LED_DIMM_DELAY 0x100000
void vLedSignalingTask(void *pvArg)
{
    ledLighting_t ledLighting = LED_BRIGHTEN;
    uint8_t ledBrightnessVal  = 0;
    portTickType xLastTimePoint   = 0;
    const portTickType xTimeDelay = 20/portTICK_RATE_MS;

    vLED_Configuration();
/*
   int i = 0;
   while(1) {

        GPIO_SetBits(GPIOB, GPIO_Pin_1);
        for(i=0; i<LED_DIMM_DELAY; i++);

        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        for(i=0; i<LED_DIMM_DELAY; i++);
    }

    return;
*/
    xLastTimePoint = xTaskGetTickCount();
    while(1) {
//        if(xSemaphoreTake(xSemaphLedSignal, 100/portTICK_RATE_MS) == pdTRUE) {
	  {
            if(ledLighting == LED_BRIGHTEN) {
                ledBrightnessVal++;
                if(ledBrightnessVal == 100U) {
                    ledLighting = LED_FADING;
                }
            }
            else {
                ledBrightnessVal--;
                if(ledBrightnessVal == 0U) {
                    ledLighting = LED_BRIGHTEN;
                }
            }

            TIM3->CCR4 = ledBrightnessVal;
            vTaskDelayUntil(&xLastTimePoint, xTimeDelay);
        }
//        else {
//            TIM3->CCR4 = 0;
//        }
    }

}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
