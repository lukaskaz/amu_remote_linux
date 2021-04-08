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

typedef enum {
    DRIVE_ST_STOPPED = 0,
    DRIVE_ST_WRN_1_RUNNING,
    DRIVE_ST_WRN_1_TO_2_RUNNING,
    DRIVE_ST_WRN_2_RUNNING,
    DRIVE_ST_FREE_RUNNING,
} driveStatus_t;

static void drive_adjust_forward_speed_on_obstacle(driveControlData_t *drivePtr);
static void drive_adjust_backward_speed_on_obstacle(driveControlData_t *drivePtr);
static driveStatus_t get_front_drive_status(driveControlData_t *drivePtr, sensorProximity_t proxNotif, driveStatus_t currDrvStatus);

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

void vDrive_motors_control(const movement_settings_t*);

void vExec_drive_update(movement_settings_t* status)
{
        vDrive_motors_control(status);
//    if(DRIVE_OP_WAIT != status->direction) {
//        vDrive_motors_control(status);
//        if(DRIVE_OP_STOPPED != status->direction) \
//                status->direction = DRIVE_OP_WAIT;
//    }
//    else status->direction = DRIVE_OP_STOPPED;
}

void vDrive_motors_control(const movement_settings_t* state)
{
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
        case DRIVE_OP_JOY_LEFT:
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
            GPIO_SetBits(GPIOA, GPIO_Pin_7);
            TIM_OC1PolarityConfig(TIM3, TIM_OCPolarity_High);
            TIM_OC3PolarityConfig(TIM3, TIM_OCPolarity_Low);
            TIM_SetCompare1(TIM3, state->speed_lft);
            TIM_SetCompare3(TIM3, state->speed_rgt);
            break;
        case DRIVE_OP_RIGHT:
        case DRIVE_OP_JOY_RIGHT:
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


    driveControlData_t drive = {0};

    vDrive_Configuration();
//    xQueueDriveControlCmd = xQueueCreate(DRIVE_DATA_QUEUE_SIZE, sizeof(driveControlData_t));

    while(1) {
//        xQueueReceive(xQueueDriveControlCmd, &drive, portMAX_DELAY);
        printf("[%s] Received item %#x!\n\r", __func__, drive.direction);
        
        if(drive.speed_0 > DRIVE_SPEED_MAX) {
            drive.speed_0 = DRIVE_SPEED_MAX;
        }
        if(drive.speed_1 > DRIVE_SPEED_MAX) {
            drive.speed_1 = DRIVE_SPEED_MAX;
        }
                
        switch(drive.direction) {
            case DRIVE_OP_STOPPED:
            case DRIVE_OP_JOY_STOPPED:
                printf("Motors stopped!\n\r");
                drive_adjust_forward_speed_on_obstacle(&drive);
                TIM3->CCR1 = 0;
                TIM3->CCR3 = 0;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_FORWARD:
            case DRIVE_OP_JOY_FORWARD:
                //printf("Move forward START\n\r");
                //printf("Move forward %d\n\r", driveOperation.speed );
                drive_adjust_forward_speed_on_obstacle(&drive);
                TIM3->CCR1 = drive.speed_0;
                TIM3->CCR3 = drive.speed_1;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                //printf("Move forward END\n\r");
                break;
            case DRIVE_OP_BACKWARD:
            case DRIVE_OP_JOY_BACKWARD:
                //printf("Move backward START\n\r");
                //printf("Move backward %d\n\r", driveOperation.speed);
                drive_adjust_backward_speed_on_obstacle(&drive);
                TIM3->CCR1 = DRIVE_SPEED_MAX - drive.speed_0;
                TIM3->CCR3 = DRIVE_SPEED_MAX - drive.speed_1;
                GPIO_SetBits(GPIOA, GPIO_Pin_5);
                GPIO_SetBits(GPIOA, GPIO_Pin_7);
                //printf("Move backward END\n\r");
                break;
            case DRIVE_OP_LEFT:
                printf("Move forward START\n\r");
                //printf("Turn left %d\n\r", driveOperation.speed);
                TIM3->CCR1 = drive.speed_0;
                TIM3->CCR3 = DRIVE_SPEED_MAX - drive.speed_0;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_SetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_RIGHT:
                printf("Move forward START\n\r");
                //printf("Turn right %d\n\r", driveOperation.speed);
                TIM3->CCR1 = DRIVE_SPEED_MAX - drive.speed_1;
                TIM3->CCR3 = drive.speed_1;
                GPIO_SetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_JOY_LEFT:
            case DRIVE_OP_JOY_RIGHT:
                //printf("Move joy TURN %d/%d\n\r", drive.speed_0, drive.speed_1);
                TIM3->CCR1 = drive.speed_0;
                TIM3->CCR3 = drive.speed_1;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                break;
            default:
                printf("Unsupported drive operation!\n\r");
                break;
        }
    }
}

static driveStatus_t get_front_drive_status(driveControlData_t *drivePtr, sensorProximity_t proxNotif, driveStatus_t currDrvStatus)
{
    driveStatus_t fresult = DRIVE_ST_STOPPED;
//    extern int8_t acclVals[];
//    extern int acclValsPos;

    if(drivePtr->direction != DRIVE_OP_STOPPED && drivePtr->direction != DRIVE_OP_JOY_STOPPED) {
        if(currDrvStatus == DRIVE_ST_STOPPED) {
            switch(proxNotif) {
                case SEN_PROX_NONE:
                    fresult = DRIVE_ST_FREE_RUNNING;
                    break;
                case SEN_PROX_WRN_1:
                    fresult = DRIVE_ST_WRN_1_RUNNING;
                    break;
                case SEN_PROX_WRN_2:
                    fresult = DRIVE_ST_WRN_2_RUNNING;
                    break;
                case SEN_PROX_ALERT:
                    fresult = DRIVE_ST_STOPPED;
                    break;
                default: {}
            }
        }
        else {
            switch(proxNotif) {
                case SEN_PROX_NONE:
                    fresult = DRIVE_ST_FREE_RUNNING;
                    break;
                case SEN_PROX_WRN_1:
                    fresult = currDrvStatus;
                    break;
                case SEN_PROX_WRN_2: {
                    if(currDrvStatus == DRIVE_ST_WRN_1_RUNNING) {
                        fresult = DRIVE_ST_WRN_1_TO_2_RUNNING;
                    }
                    else if( currDrvStatus == DRIVE_ST_FREE_RUNNING      || 
                             currDrvStatus == DRIVE_ST_WRN_1_TO_2_RUNNING )
                    {
//                        if(is_robot_stopped() == true) {
//                                fresult = DRIVE_ST_WRN_2_RUNNING;
//                                //acclVals[acclValsPos++] = -55;
//                        }
//                        else {
//                            fresult = currDrvStatus;
//                        }
                    }
                    else {
                        fresult = currDrvStatus;
                    }
                    break;
                }
                case SEN_PROX_ALERT:
                    fresult = DRIVE_ST_STOPPED;
                    break;
                default: {}
            }
        }
    }
    else {
        // if stop case occurs during movement do not set stopped state
//        if(currDrvStatus != DRIVE_ST_STOPPED && is_robot_stopped() != true) {
//            //acclVals[acclValsPos++] = -66;
//            fresult = currDrvStatus;
//        }
    }

//    acclVals[acclValsPos++] = (-1)*currDrvStatus;
//    acclVals[acclValsPos++] = fresult;
//    acclVals[acclValsPos++] = proxNotif;
    return fresult;
}

static void drive_adjust_forward_speed_on_obstacle(driveControlData_t *drivePtr)
{
    static driveStatus_t driveStatus = DRIVE_ST_STOPPED;
    sensorProximity_t proxNotification = get_front_proximity_estimation();

    driveStatus = get_front_drive_status(drivePtr, proxNotification, driveStatus);
    if(proxNotification == SEN_PROX_WRN_1) {
        if(driveStatus == DRIVE_ST_FREE_RUNNING) {
            if(drivePtr->speed_0 > 40) {
                drivePtr->speed_0 = 40;
            }
            if(drivePtr->speed_1 > 40) {
                drivePtr->speed_1 = 40;
            }
        }
        else if(driveStatus == DRIVE_ST_WRN_1_RUNNING) {
            if(drivePtr->speed_0 > 70) {
                drivePtr->speed_0 = 70;
            }
            if(drivePtr->speed_1 > 70) {
                drivePtr->speed_1 = 70;
            }
        }
        else {
            //unsupported case, do nothing
        }
        printf("Obstacle detected: %d, %d, %d\r\n", proxNotification, drivePtr->speed_0, drivePtr->speed_1);
    }
    else if(proxNotification == SEN_PROX_WRN_2) {
        if(driveStatus == DRIVE_ST_FREE_RUNNING) {
            if(drivePtr->speed_0 > 10) {
                drivePtr->speed_0 = 10;
            }
            if(drivePtr->speed_1 > 10) {
                drivePtr->speed_1 = 10;
            }
        }
        else if(driveStatus == DRIVE_ST_WRN_1_TO_2_RUNNING) {
            if(drivePtr->speed_0 > 40) {
                drivePtr->speed_0 = 40;
            }
            if(drivePtr->speed_1 > 40) {
                drivePtr->speed_1 = 40;
            }
        }
        else if(driveStatus == DRIVE_ST_WRN_2_RUNNING) {
            if(drivePtr->speed_0 > 60) {
                drivePtr->speed_0 = 60;
            }
            if(drivePtr->speed_1 > 60) {
                drivePtr->speed_1 = 60;
            }
        }
        else {
            // unsupported case, do nothing
        }
        printf("Obstacle near detected: %d, %d, %d\r\n", proxNotification, drivePtr->speed_0, drivePtr->speed_1);
    }
    else if(proxNotification == SEN_PROX_ALERT) {
        drivePtr->speed_0 = 0;
        drivePtr->speed_1 = 0;

        printf("Obstacle very near detected: %d, %d, %d\r\n", proxNotification, drivePtr->speed_0, drivePtr->speed_1);
    }
    else {
        // no obstacle detected nearby
        driveStatus = DRIVE_ST_FREE_RUNNING;
    }

}

static void drive_adjust_backward_speed_on_obstacle(driveControlData_t *drivePtr)
{
    
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
