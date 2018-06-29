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

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mod_drive.h"
#include "mod_console.h"
#include "mod_signaling_led.h"
#include "mod_radio_control.h"
#include "mod_sensors.h"
#include "mod_orientation_sensor.h"
#include "mod_lcd.h"
#include "mod_i2c.h"
#include "mod_sound_signal.h"
#include "mod_lighting.h"


/* Private define ------------------------------------------------------------*/

#define RADIO_CONTROL_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE )
#define RADIO_CONTROL_TASK_PRIORITY       ( tskIDLE_PRIORITY + 3 )

#define DRIVE_CONTROL_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE )
#define DRIVE_CONTROL_TASK_PRIORITY       ( tskIDLE_PRIORITY + 3 )

#define LED_SIGNALING_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE )
#define LED_SIGNALING_TASK_PRIORITY       ( tskIDLE_PRIORITY + 1 )

#define CONSOLE_INTERFACE_TASK_STACK_SIZE ( configMINIMAL_STACK_SIZE )
#define CONSOLE_INTERFACE_TASK_PRIORITY   ( tskIDLE_PRIORITY + 1 )

#define SENSORS_SERVICE_TASK_STACK_SIZE   ( configMINIMAL_STACK_SIZE )
#define SENSORS_SERVICE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2 )

#define ORSENSORS_SERVICE_TASK_STACK_SIZE ( configMINIMAL_STACK_SIZE )
#define ORSENSORS_SERVICE_TASK_PRIORITY   ( tskIDLE_PRIORITY + 2 )

#define LCD_INTERFACE_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE )
#define LCD_INTERFACE_TASK_PRIORITY       ( tskIDLE_PRIORITY + 2 )


/* Private function prototypes -----------------------------------------------*/
static void prvSetupHardware(void);
static void vRCC_configuration(void);
static void vNVIC_configuration(void);

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//static char test77[sizeof( StackType_t ) * CONSOLE_INTERFACE_TASK_STACK_SIZE] = {0};

int main(void)
{
//    _REENT_INIT_PTR(_REENT);

    prvSetupHardware();



//    vConsoleInterfaceTask(NULL);

//    vLedSignalingTask(NULL);  

  //  test77[0] = '\0';

//    xTaskCreate(vRadio_Control, (signed char *) "Radio control",
//        RADIO_CONTROL_TASK_STACK_SIZE, NULL, RADIO_CONTROL_TASK_PRIORITY, NULL);
        
//    xTaskCreate(vDrive_Control, (signed char *) "Drive control",
//        DRIVE_CONTROL_TASK_STACK_SIZE, NULL, DRIVE_CONTROL_TASK_PRIORITY, NULL);
        
    xTaskCreate(vLedSignalingTask, (char*) "LED Signaling",
        LED_SIGNALING_TASK_STACK_SIZE, NULL, LED_SIGNALING_TASK_PRIORITY, NULL);
        
    xTaskCreate(vConsoleInterfaceTask, (char*) "Console",
        CONSOLE_INTERFACE_TASK_STACK_SIZE, NULL, CONSOLE_INTERFACE_TASK_PRIORITY, NULL);


//    xTaskCreate(vSensorsServiceTask, (signed char *) "Sensors",
//        SENSORS_SERVICE_TASK_STACK_SIZE, NULL, SENSORS_SERVICE_TASK_PRIORITY, NULL);
    
    xTaskCreate(vOrientSensorServiceTask, (signed char *) "Orinent sensors",
        ORSENSORS_SERVICE_TASK_STACK_SIZE, NULL, ORSENSORS_SERVICE_TASK_PRIORITY, NULL);

    xTaskCreate(vLcdInterfaceTask, (signed char *) "LCD",
        LCD_INTERFACE_TASK_STACK_SIZE, NULL, LCD_INTERFACE_TASK_PRIORITY, NULL);

        
    /* Start the scheduler. */
    vTaskStartScheduler();

    return 0;
}

/* Defining malloc/free should overwrite the standard versions provided by the compiler. */ 
//void* malloc (size_t size) 
//{ 
//	/* Call the FreeRTOS version of malloc. */ 
//	return pvPortMalloc( size ); 
//} 
//
//void free (void* ptr) 
//{ 
//	/* Call the FreeRTOS version of free. */ 
//	vPortFree(ptr); 
//}

/*
char* _sbrk(size_t x)
{
    ( void ) x;
    return NULL;
}


int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

void _exit(int status)
{
    while(1);
}

int _close(int file) 
{
    return 0;
}

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}
*/

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

    xQueueDriveControlCmd = xQueueCreate(4, sizeof(driveControlData_t));
    vSemaphoreCreateBinary(xSemaphLedSignal);
    vSemaphoreCreateBinary(xSemaphI2CLcdInitDone);
    vSemaphoreCreateBinary(xSemaphRadioPacketReady);

    // semaphores are in ready state at start, so to release them have use take
    xSemaphoreTake(xSemaphLedSignal, portMAX_DELAY);
    xSemaphoreTake(xSemaphI2CLcdInitDone, portMAX_DELAY);
    xSemaphoreTake(xSemaphRadioPacketReady, portMAX_DELAY);
    
    vRCC_configuration();
    vNVIC_configuration();
    vUSB_configuration();
    vI2C_configuration();
    vSound_configuration();
    vLighting_configuration();


}

static void vRCC_configuration(void) 
{ 
    /* configuration of clock signals */
    ErrorStatus HSEStartUpStatus; 
    
    /* RCC full reset */
    RCC_DeInit(); 
    
    /* activate HSE (External High Speed oscillator): 8 MHz */                                                        
    RCC_HSEConfig(RCC_HSE_ON);                                      
    HSEStartUpStatus = RCC_WaitForHSEStartUp();  
                 
    if(HSEStartUpStatus == SUCCESS) 
    { 
        /* set up FLASH */
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /* configure prescalers */                                                                    
        RCC_HCLKConfig(RCC_SYSCLK_Div1);                          //set HCLK=SYSCLK 
        RCC_PCLK2Config(RCC_HCLK_Div1);                           //set PCLK2=HCLK 
        RCC_PCLK1Config(RCC_HCLK_Div2);                           //set PCLK1=HCLK/2
        RCC_ADCCLKConfig(RCC_PCLK2_Div6);                         //adjust ADC frequency to 72MHz/6 = 12MHz
        /* Select USBCLK source for 48 MHz to USB OTG FS*/
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);          //adjust USB frequency to 72MHz/1.5 = 48MHz
       
        /* set up PLL for max nominal speed 72 MHz*/
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);      //set PLLCLK = HSE*9 -> 8MHz*9 = 72 MHz 
        
        /* Activate PLL */
        RCC_PLLCmd(ENABLE);                                       
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        /* set up SYSCLK */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);                 //SYSCLK = PLLCLK => 72 MHz
        while(RCC_GetSYSCLKSource() != 0x08);
        
        /* Configure HCLK clock as SysTick clock source. */
        SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
    } 
    else 
    { 
        /* signalization of RCC startup malfunction: STATUS LED*/
        while(1);
    }                     
}

static void vNVIC_configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // interruption for radio RF usart
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //interruption for USB
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // interruption for distance sensor time measurements
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // interruption for ADC external trigger
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // interruption for power supply voltage detector
    NVIC_InitStructure.NVIC_IRQChannel                   = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
        /* This function will get called if a task overflows its stack.   If the
 *         parameters are corrupt then inspect pxCurrentTCB to find which was the
 *                 offending task. */

        ( void ) pxTask;
        ( void ) pcTaskName;

                USB_Send_Data("Buffer overflow!!!", 19);
                //vTaskDelay(1000);
                                                

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
