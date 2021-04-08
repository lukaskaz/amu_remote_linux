/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               main.c
** Descriptions:            The FreeRTOS application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-8
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

/* Includes ------------------------------------------------------------------*/
#include  <stdio.h>

#include "mod_radio.h"
#include "stm32f10x_it.h"
#include "stm32f10x_conf.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mod_drive.h"
#include "mod_console.h"
#include "mod_led.h"
#include "mod_sensors.h"
#include "mod_orientsensor.h"
#include "mod_lcd.h"
#include "mod_i2c.h"
#include "mod_sound.h"
#include "mod_lighting.h"
#include "mod_auxiliary.h"


/* Private define ------------------------------------------------------------*/
#define SIZE_TO_WORDS(bytes)        ( unsigned short )(bytes/4)

#define RADIO_TASK_STACK_SIZE       ( configMINIMAL_STACK_SIZE )
#define RADIO_TASK_PRIORITY         ( tskIDLE_PRIORITY + 4     )
#define DRIVE_TASK_STACK_SIZE       ( configMINIMAL_STACK_SIZE )
#define DRIVE_TASK_PRIORITY         ( tskIDLE_PRIORITY + 4     )
#define LED_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE )
#define LED_TASK_PRIORITY           ( tskIDLE_PRIORITY + 1     )
#define CONSOLE_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE )
#define CONSOLE_TASK_PRIORITY       ( tskIDLE_PRIORITY + 1     )
#define SENSORS_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE )
#define SENSORS_TASK_PRIORITY       ( tskIDLE_PRIORITY + 3     )
#define LCD_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE )
#define LCD_TASK_PRIORITY           ( tskIDLE_PRIORITY + 2     )


/* Private function prototypes -----------------------------------------------*/
static void prvSetupObjects(void);
static void prvSetupHardware(void);
static void vNVIC_configuration(void);

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int main(void)
{
    _REENT_INIT_PTR(_REENT);

    prvSetupObjects();
    prvSetupHardware();

    xTaskCreate(vLedTask, "LED", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
    xTaskCreate(vRadioTask, "Radio", RADIO_TASK_STACK_SIZE, NULL, RADIO_TASK_PRIORITY, NULL);
    xTaskCreate(vDriveTask, "Drive", DRIVE_TASK_STACK_SIZE, NULL, DRIVE_TASK_PRIORITY, NULL);
    xTaskCreate(vConsoleTask, "Console", CONSOLE_TASK_STACK_SIZE, NULL, CONSOLE_TASK_PRIORITY, NULL);
    xTaskCreate(vSensorsTask, "Sensors", SENSORS_TASK_STACK_SIZE, NULL, SENSORS_TASK_PRIORITY, NULL);
    xTaskCreate(vLcdTask, "LCD", LCD_TASK_STACK_SIZE, NULL, LCD_TASK_PRIORITY, NULL);
    vTaskStartScheduler();

    return 0;
}

/*******************************************************************************
* Function Name  : prvSetupObjects
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void prvSetupObjects(void)
{
    sys_data_mutex = xSemaphoreCreateMutex();
}

/*******************************************************************************
* Function Name  : prvSetupHardware
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void prvSetupHardware(void)
{

    vNVIC_configuration();
    vUSB_configuration();
    vI2C_configuration();
    vDrive_Configuration();
    vSound_configuration();
    vLighting_configuration();
}

#define FREERTOS_USER_IRQS_GROUPING         NVIC_PriorityGroup_4
#define FREERTOS_USER_IRQ_PRIORITY_MAX      11
#define FREERTOS_USER_IRQ_PRIORITY_MIN      15
static void vNVIC_configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = \
        {.NVIC_IRQChannelSubPriority = 0, .NVIC_IRQChannelCmd = ENABLE };

    NVIC_PriorityGroupConfig(FREERTOS_USER_IRQS_GROUPING);
    // interruption setup for radio RF usart
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = \
                                         FREERTOS_USER_IRQ_PRIORITY_MAX;
    NVIC_Init(&NVIC_InitStructure);

    //interruption setup for USB
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = \
                                         FREERTOS_USER_IRQ_PRIORITY_MAX + 1;
    NVIC_Init(&NVIC_InitStructure);
    
    // interruption setup for led signaling
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = \
                                         FREERTOS_USER_IRQ_PRIORITY_MIN;
    NVIC_Init(&NVIC_InitStructure);

    // interruption setup for distance sensor time measurements
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = \
                                         FREERTOS_USER_IRQ_PRIORITY_MAX + 1;
    NVIC_Init(&NVIC_InitStructure);

    // interruption setup for power supply voltage detector
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = \
                                         FREERTOS_USER_IRQ_PRIORITY_MAX + 2;
    NVIC_Init(&NVIC_InitStructure);
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
        /* This function will get called if a task overflows its stack.   If the
 *         parameters are corrupt then inspect pxCurrentTCB to find which was the
 *                 offending task. */

        ( void ) pxTask;
        ( void ) pcTaskName;

                printf("Buffer overflow!!!");
                //vTaskDelay(pdMS_TO_TICKS(1000));
                                                

        for( ;; );
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
