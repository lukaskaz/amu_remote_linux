/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_drive.c
** Descriptions:            Interface for controling dc motors drive
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
#include <stdint.h>
#include <stdbool.h>

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mod_sensors.h"
#include "mod_lighting.h"
#include "mod_sound.h"
#include "mod_drive.h"

#define DRIVE_TASK_INTERVAL_MS      50
#define DRIVE_PWM_TIMER_STEPS       DRIVE_SPEED_MAX
#define DRIVE_PWM_TIMER_FREQ        500

static void drive_mod_speed_on_front_prox(movement_settings_t *);

void vDrive_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    RCC_ClocksTypeDef RCC_Clocks = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_GetClocksFreq(&RCC_Clocks);
    TIM_TimeBaseStructure.TIM_Period = \
        TIMER_STEPS_TO_PERIOD(DRIVE_PWM_TIMER_STEPS);
    TIM_TimeBaseStructure.TIM_Prescaler = \
        TIMER_FREQHZ_TO_PRESCAL(RCC_Clocks, DRIVE_PWM_TIMER_STEPS, DRIVE_PWM_TIMER_FREQ);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse  = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void vExec_lighting_update(light_settings_t* status)
{
    vLighting_set_auto(status->autodetect);
    if(true == bLighting_is_auto()) {
        vLighting_adjust_auto_light();
        *status = vLighting_get_state();
    }
    else vLighting_set_state(status);
}

void vExec_signal_update(signal_settings_t* status)
{
    status->honk = SOUND_RF_PRESS == status->honk ? SOUND_RF_WAIT:
                    SOUND_RF_WAIT == status->honk ? SOUND_RF_NONE:status->honk;
    vSound_Signal_RF_Control(SOUND_RF_NONE != status->honk ? \
                                    SOUND_RF_PRESS:SOUND_RF_NONE);
}

void vDrive_motors_control(movement_settings_t*);

void vExec_drive_update(movement_settings_t* status)
{
    if(DRIVE_OP_WAIT != status->direction) {
        vDrive_motors_control(status);
        if(DRIVE_OP_STOPPED != status->direction) \
            status->direction = DRIVE_OP_WAIT;
    }
    else status->direction = DRIVE_OP_STOPPED;
}

void vDrive_motors_control(movement_settings_t* state)
{
//    drive_mod_speed_on_front_prox(state);
    switch(state->direction) {
        case DRIVE_OP_FORWARD:
        case DRIVE_OP_JOY_FORWARD:
            GPIO_ResetBits(GPIOA, GPIO_Pin_5|GPIO_Pin_7);
            TIM_OC1PolarityConfig(TIM3, TIM_OCPolarity_High);
            TIM_OC3PolarityConfig(TIM3, TIM_OCPolarity_High);
            TIM_SetCompare1(TIM3, state->speed_lft);
            TIM_SetCompare3(TIM3, state->speed_rgt);
            break;
        case DRIVE_OP_BACKWARD:
        case DRIVE_OP_JOY_BACKWARD:
            GPIO_SetBits(GPIOA, GPIO_Pin_5|GPIO_Pin_7);
            TIM_OC1PolarityConfig(TIM3, TIM_OCPolarity_Low);
            TIM_OC3PolarityConfig(TIM3, TIM_OCPolarity_Low);
            TIM_SetCompare1(TIM3, state->speed_lft);
            TIM_SetCompare3(TIM3, state->speed_rgt);
            break;
        case DRIVE_OP_LEFT:
        case DRIVE_OP_JOY_FW_LEFT:
        case DRIVE_OP_JOY_BW_RIGHT:
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
            GPIO_SetBits(GPIOA, GPIO_Pin_7);
            TIM_OC1PolarityConfig(TIM3, TIM_OCPolarity_High);
            TIM_OC3PolarityConfig(TIM3, TIM_OCPolarity_Low);
            TIM_SetCompare1(TIM3, state->speed_lft);
            TIM_SetCompare3(TIM3, state->speed_rgt);
            break;
        case DRIVE_OP_RIGHT:
        case DRIVE_OP_JOY_FW_RIGHT:
        case DRIVE_OP_JOY_BW_LEFT:
            GPIO_SetBits(GPIOA, GPIO_Pin_5);
            GPIO_ResetBits(GPIOA, GPIO_Pin_7);
            TIM_OC1PolarityConfig(TIM3, TIM_OCPolarity_Low);
            TIM_OC3PolarityConfig(TIM3, TIM_OCPolarity_High);
            TIM_SetCompare1(TIM3, state->speed_lft);
            TIM_SetCompare3(TIM3, state->speed_rgt);
            break;
        case DRIVE_OP_JOY_STOPPED:
        case DRIVE_OP_STOPPED: //no break
        default:
            GPIO_ResetBits(GPIOA, GPIO_Pin_5|GPIO_Pin_7);
            TIM_OC1PolarityConfig(TIM3, TIM_OCPolarity_High);
            TIM_OC3PolarityConfig(TIM3, TIM_OCPolarity_High);
            TIM_SetCompare1(TIM3, 0);
            TIM_SetCompare3(TIM3, 0);
            break;
    }
}

void vDriveTask(void *pvArg)
{
    TickType_t xLastTimePoint = 0;
    const TickType_t interval = pdMS_TO_TICKS(DRIVE_TASK_INTERVAL_MS);

    light_set_auto(bLighting_is_auto());
    signal_set_state(true == vSound_is_signal() ? SOUND_RF_PRESS:SOUND_RF_NONE);
    while(1) {
        exec_settings_t status = exec_group_get_status();
        vExec_drive_update(&status.movement);
        vExec_lighting_update(&status.light);
        vExec_signal_update(&status.signal);
        exec_group_set_status(&status);

        vTaskDelayUntil(&xLastTimePoint, interval);
    }
}

#define IS_DRIVE_RUNNING_FORWARD(dat)   \
    ((0 < dat->speed_lft || 0 < dat->speed_rgt) && \
     (DRIVE_OP_FORWARD == dat->direction || DRIVE_OP_JOY_FORWARD == dat->direction || \
      DRIVE_OP_LEFT == dat->direction || DRIVE_OP_RIGHT == dat->direction   || \
      DRIVE_OP_JOY_FW_LEFT == dat->direction || DRIVE_OP_JOY_FW_RIGHT == dat->direction))
#define DRIVE_LIMIT_SPEED(spd,limit) spd = limit<spd ? limit:spd;

#define DRIVE_PROX_WARN_SPEED_MAX    75
#define DRIVE_PROX_ALERT_SPEED_MAX   0
static void drive_mod_speed_on_front_prox(movement_settings_t *mov)
{
    if(true == IS_DRIVE_RUNNING_FORWARD(mov)) {
        sensorProximity_t prox_info = get_front_proximity_estimation();
        if(SEN_PROX_WARN == prox_info) {
            DRIVE_LIMIT_SPEED(mov->speed_lft, DRIVE_PROX_WARN_SPEED_MAX);
            DRIVE_LIMIT_SPEED(mov->speed_rgt, DRIVE_PROX_WARN_SPEED_MAX);
        }
        else {
            if(SEN_PROX_ALERT == prox_info) {
                DRIVE_LIMIT_SPEED(mov->speed_lft, DRIVE_PROX_ALERT_SPEED_MAX);
                DRIVE_LIMIT_SPEED(mov->speed_rgt, DRIVE_PROX_ALERT_SPEED_MAX);
            }
        }
    }
}

static void drive_mod_speed_on_back_prox(driveControlData_t *drivePtr)
{
    
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
