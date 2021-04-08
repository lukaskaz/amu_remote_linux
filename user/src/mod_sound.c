/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_sound_signal.c
** Descriptions:            Interface for sound signaling
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
#include <stdbool.h>

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mod_sound.h"

#define SOUND_GET_STATE         GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)
#define SOUND_SET_STATE(state)  GPIO_WriteBit(GPIOC, GPIO_Pin_13, state)
#define SOUND_TOGGLE_STATE      SOUND_SET_STATE(!SOUND_GET_STATE)
#define SOUND_ENABLE            SOUND_SET_STATE(Bit_RESET);
#define SOUND_DISABLE           SOUND_SET_STATE(Bit_SET);

typedef enum {
    SOUND_OP_NONE = '0',
    SOUND_OP_PLAIN,
} SoundOperation_t;

void vSound_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void vSound_Signal_Control(const uint8_t state)
{
    enum { SOUND_NONE = 0, SOUND_PRESS, SOUND_TOGGLE };
    switch(state) {
        case SOUND_TOGGLE: SOUND_TOGGLE_STATE; break;
        case SOUND_PRESS: SOUND_ENABLE; vTaskDelay(pdMS_TO_TICKS(50));
        //no break
        case SOUND_NONE: SOUND_DISABLE; break;
        default: break;
    }
}

void vSound_Signal_RF_Control(const uint8_t status)
{
    switch(status) {
        case SOUND_RF_NONE: SOUND_DISABLE; break;
        case SOUND_RF_PRESS: SOUND_ENABLE; break;
        default: printf("Sound operation not supported!\n\r");
    }
}

bool vSound_is_signal(void)
{
    return Bit_RESET == SOUND_GET_STATE ? true:false;
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
