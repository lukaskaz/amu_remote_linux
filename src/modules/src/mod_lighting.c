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

#include "mod_lighting.h"
#include "stm32f10x.h"
#include "mod_lcd.h"
#include "mod_sensors.h"

typedef enum {
    LIGHT_MODE_MANUAL = 0,
    LIGHT_MODE_AUTO,
} lightMode_t;

typedef enum {
    LIGHT_OP_NONE = '0',
    LIGHT_OP_LEFT,
    LIGHT_OP_RIGHT,
    LIGHT_OP_INNER,
    LIGHT_OP_OUTER,
} LightOperation_t;

typedef enum {
    LIGHT_RF_NONE  = 0x00,
    LIGHT_RF_LEFT  = 0x01,
    LIGHT_RF_RIGHT = 0x02,
    LIGHT_RF_INNER = 0x04,
    LIGHT_RF_OUTER = 0x08,
} LightRFOperation_t;

typedef enum {
    LIGHT_RF_DISABLE = 0,
    LIGHT_RF_ENABLE,
} LightState_t;

static lightMode_t currentLightMode = LIGHT_MODE_AUTO;

static void resetLightingToDefaults(void);
static void adjustLightingMode(void);
static void vLighting_Control(const uint8_t state);

static void resetLightingToDefaults(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_4);
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
    GPIO_SetBits(GPIOB, GPIO_Pin_3);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

bool isLightingAutoModeEnabled(void)
{
    bool fresult = false;

    if(currentLightMode == LIGHT_MODE_AUTO) {
        fresult = true;
    }
    
    return fresult;
}

static void adjustLightingMode(void)
{
    lcdControlData_t lcdData = {0};

    if(currentLightMode == LIGHT_MODE_MANUAL) {
        if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) == Bit_SET &&
           GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5) == Bit_SET &&
           GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) == Bit_SET &&
           GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) == Bit_SET ) {
            currentLightMode = LIGHT_MODE_AUTO;
            lcdData.operation = LCD_OP_LIGHTING;
            lcdData.state = LCD_LIGHT_AUTO;
        }
    }
    else {
        currentLightMode = LIGHT_MODE_MANUAL;
        lcdData.operation = LCD_OP_LIGHTING;
        lcdData.state = LCD_LIGHT_MANUAL;
        //adjust_auto_lighting();
    }
    
    if(lcdData.operation != LCD_OP_NONE) {
        //xQueueSend(xQueueLcdControl, (void *)&lcdData, 0);
    }
}

void vLighting_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);

    /* SWJ Disabled (JTAG-DP + SW-DP) and I/O Pins enabled instead */
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

void vLighting_Console(void)
{
    char selection = 0;

    printf("\n\r"
           "--------------------------\n\r"
           "|    SOUND SIGNAL MENU   |\n\r"
           "--------------------------\n\r"
           "| 1. switch left led     |\n\r"
           "| 2. switch right led    |\n\r"
           "| 3. switch inner leds   |\n\r"
           "| 4. switch outer leds   |\n\r"
           "|                        |\n\r"
           "| 0. back to main menu   |\n\r"
           "--------------------------\n\r"
           " Selection/> ");

    do {
        //GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        selection = getchar();

        vLighting_Control(selection);

    } while(selection != '0');
}

static void vLighting_Control(const uint8_t state)
{
    switch(state) {
        case LIGHT_OP_LEFT:
            if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) == Bit_SET) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_4);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_4);
            }
            break;
        case LIGHT_OP_RIGHT:
            if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5) == Bit_SET) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_5);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_5);
            }
            break;
        case LIGHT_OP_INNER:
            if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) == Bit_SET) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
            }
            break;
        case LIGHT_OP_OUTER:
            if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) == Bit_SET) {
                GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            }
            else {
                GPIO_SetBits(GPIOA, GPIO_Pin_15);
            }
            break;
        default:
            printf("Lighting operation not supported!\n\r");
    }
}

void vGet_Lighting_State(uint8_t* state)
{
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET) {
        *state |= LIGHT_RF_LEFT;
    }

    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5) == Bit_RESET) {
        *state |= LIGHT_RF_RIGHT;
    }

    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) == Bit_RESET) {
        *state |= LIGHT_RF_INNER;
    }

    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET) {
        *state |= LIGHT_RF_OUTER;
    }
}

void vLighting_RF_Control(const uint8_t type, const uint8_t state)
{
    //printf("Lighting %d/%d/r/n", type, state);
    if(isLightingAutoModeEnabled() == true) {
        resetLightingToDefaults();
    }

    switch(type) {
        case LIGHT_RF_LEFT:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_4);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_4);
            }
            break;
        case LIGHT_RF_RIGHT:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_5);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_5);
            }
            break;
        case LIGHT_RF_INNER:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
            }
            break;
        case LIGHT_RF_OUTER:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            }
            else {
                GPIO_SetBits(GPIOA, GPIO_Pin_15);
            }
            break;
        default:
            printf("Lighting operation not supported!\n\r");
    }

    adjustLightingMode();
}

void vLighting_Auto_Control(const LightAutoOperation_t type)
{
    switch(type) {
        case LIGHT_AUTO_NONE:
            GPIO_SetBits(GPIOB, GPIO_Pin_4);
            GPIO_SetBits(GPIOB, GPIO_Pin_5);
            GPIO_SetBits(GPIOB, GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            break;
        case LIGHT_AUTO_MIN:
            GPIO_ResetBits(GPIOB, GPIO_Pin_4);
            GPIO_ResetBits(GPIOB, GPIO_Pin_5);
            GPIO_SetBits(GPIOB, GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            break;
        case LIGHT_AUTO_MEDIUM:
            GPIO_ResetBits(GPIOB, GPIO_Pin_4);
            GPIO_ResetBits(GPIOB, GPIO_Pin_5);
            GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            break;
        case LIGHT_AUTO_MAX:
            GPIO_ResetBits(GPIOB, GPIO_Pin_4);
            GPIO_ResetBits(GPIOB, GPIO_Pin_5);
            GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            break;
        default: {}
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
