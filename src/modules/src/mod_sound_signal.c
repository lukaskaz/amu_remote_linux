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

#include "mod_sound_signal.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

typedef enum {
    SOUND_OP_NONE = '0',
    SOUND_OP_PLAIN,
} SoundOperation_t;

static void vSound_Signal_Control(const uint8_t status);


void vSound_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void vSound_Signal_Console(void)
{
    char selection = 0;

    printf("\n\r"
           "--------------------------\n\r"
           "|    SOUND SIGNAL MENU   |\n\r"
           "--------------------------\n\r"
           "| 1. sound horn          |\n\r"
           "|                        |\n\r"
           "| 0. back to main menu   |\n\r"
           "--------------------------\n\r"
           " Selection/> ");

    do {
        selection = getchar();
        vSound_Signal_Control(selection);

    } while(selection != '0');
}

static void vSound_Signal_Control(const uint8_t status)
{
    switch(status) {
        case SOUND_OP_PLAIN:
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            break;
        default:
            printf("Sound operation not supported!\n\r");
    }

    vTaskDelay(100);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void vSound_Signal_RF_Control(const uint8_t status)
{
    switch(status) {
        case SOUND_RF_NONE:
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            break;
        case SOUND_RF_PLAIN:
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            break;
        default:
            printf("Sound operation not supported!\n\r");
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
