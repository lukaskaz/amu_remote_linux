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
#include <stdbool.h>

#include "mod_drive.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mod_radio_control.h"
#include "mod_sensors.h"
#include "mod_orientation_sensor.h"


#define DRIVE_DATA_QUEUE_SIZE    20U
#define MAX_VELOCITY_VALUE       100U

typedef enum {
    DRIVE_ST_STOPPED = 0,
    DRIVE_ST_WRN_1_RUNNING,
    DRIVE_ST_WRN_1_TO_2_RUNNING,
    DRIVE_ST_WRN_2_RUNNING,
    DRIVE_ST_FREE_RUNNING,
} driveStatus_t;

xQueueHandle xQueueDriveControlCmd = NULL;

void vDrive_Configuration(void);
static void drive_adjust_forward_speed_on_obstacle(driveControlData_t *drivePtr);
static void drive_adjust_backward_speed_on_obstacle(driveControlData_t *drivePtr);
static driveStatus_t get_front_drive_status(driveControlData_t *drivePtr, sensorProximity_t proxNotif, driveStatus_t currDrvStatus);

void vDrive_Configuration(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
    RCC_ClocksTypeDef        RCC_Clocks;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_GetClocksFreq(&RCC_Clocks);
    TIM_TimeBaseStructure.TIM_Period        = 100U - 1U;
    TIM_TimeBaseStructure.TIM_Prescaler     = RCC_Clocks.HCLK_Frequency/(100U*100U) - 1U;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    /* TIM_SIG_PWM_TIMER enable counter */ 
    TIM_Cmd(TIM3, ENABLE);
    TIM3->CCR3 = 0;
    TIM3->CCR1 = 0;
}

#define DRIVE_CONSOLE_MENU_TEXT    "\n\r"                           \
                                   "--------------------------\n\r" \
                                   "|     MOVEMENT MENU      |\n\r" \
                                   "--------------------------\n\r" \
                                   "| 1. move forward        |\n\r" \
                                   "| 2. move backward       |\n\r" \
                                   "| 7. turn left           |\n\r" \
                                   "| 8. move right          |\n\r" \
                                   "| 9. set speed           |\n\r" \
                                   "|                        |\n\r" \
                                   "| 0. back to main menu   |\n\r" \
                                   "--------------------------\n\r" \
                                   " Selection/> "



void vDrive_Console(void)
{
    char selection = 0;
    static uint32_t velocity = 0;
    driveControlData_t driveControlData = {0};

    printf(DRIVE_CONSOLE_MENU_TEXT);
    do {
        selection = getchar();
        switch(selection) {
            case '0':
                printf("\Exiting menu %s\n\r", __func__);
                break;
            case '1':
                driveControlData.direction = DRIVE_OP_FORWARD;
                driveControlData.speed_0 = velocity;
                driveControlData.speed_1 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '2':
                driveControlData.direction = DRIVE_OP_BACKWARD;
                driveControlData.speed_0 = velocity;
                driveControlData.speed_1 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '7':
                driveControlData.direction = DRIVE_OP_LEFT;
                driveControlData.speed_0 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '8':
                driveControlData.direction = DRIVE_OP_RIGHT;
                driveControlData.speed_1 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '9': 
                printf("Present speed: %d\n\rChange to: ", velocity);
                scanf("%d", (uint32_t *)&velocity);
                printf("\n\rSet speed: %d\n\r", velocity);
                break;
            default:
                printf("Movement operation not supported %#x!\n\r", selection);
    		printf(DRIVE_CONSOLE_MENU_TEXT);
		break;
        }
    } while(selection != '0');
}

void vDrive_Control(void *pvArg)
{
    driveControlData_t drive = {0};

    vDrive_Configuration();
//    xQueueDriveControlCmd = xQueueCreate(DRIVE_DATA_QUEUE_SIZE, sizeof(driveControlData_t));

    while(1) {
        xQueueReceive(xQueueDriveControlCmd, &drive, portMAX_DELAY);
        printf("[%s] Received item %#x!\n\r", __func__, drive.direction);
        
        if(drive.speed_0 > MAX_VELOCITY_VALUE) {
            drive.speed_0 = MAX_VELOCITY_VALUE;
        }
        if(drive.speed_1 > MAX_VELOCITY_VALUE) {
            drive.speed_1 = MAX_VELOCITY_VALUE;
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
                TIM3->CCR1 = MAX_VELOCITY_VALUE - drive.speed_0;
                TIM3->CCR3 = MAX_VELOCITY_VALUE - drive.speed_1;
                GPIO_SetBits(GPIOA, GPIO_Pin_5);
                GPIO_SetBits(GPIOA, GPIO_Pin_7);
                //printf("Move backward END\n\r");
                break;
            case DRIVE_OP_LEFT:
                printf("Move forward START\n\r");
                //printf("Turn left %d\n\r", driveOperation.speed);
                TIM3->CCR1 = drive.speed_0;
                TIM3->CCR3 = MAX_VELOCITY_VALUE - drive.speed_0;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_SetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_RIGHT:
                printf("Move forward START\n\r");
                //printf("Turn right %d\n\r", driveOperation.speed);
                TIM3->CCR1 = MAX_VELOCITY_VALUE - drive.speed_1;
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
    extern int acclValsPos;

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
                        if(is_robot_stopped() == true) {
                                fresult = DRIVE_ST_WRN_2_RUNNING;
                                //acclVals[acclValsPos++] = -55;
                        }
                        else {
                            fresult = currDrvStatus;
                        }
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
        if(currDrvStatus != DRIVE_ST_STOPPED && is_robot_stopped() != true) {
            //acclVals[acclValsPos++] = -66;
            fresult = currDrvStatus;
        }
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
