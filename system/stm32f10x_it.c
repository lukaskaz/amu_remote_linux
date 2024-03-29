/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "mod_led.h"
#include "mod_radio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_istr.h"

#include "mod_sensors.h"

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
// declared in port.c FreeRTOS file
//void PendSV_Handler(void)
//{
//}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
// declared in port.c FreeRTOS file
//void SysTick_Handler(void)
//{
//}

/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    USB_Istr();
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles interrupt events generated by USART1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
    if(SET == USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
        USART_ClearFlag(USART1, USART_FLAG_RXNE);
        uint16_t data = USART_ReceiveData(USART1);
        radio_recv_handler_irq(data);
    }

}

/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles Capture Compare Events generated by
                   Timer1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void) 
{
    if(SET == TIM_GetITStatus(TIM1, TIM_IT_CC3)) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
        led_update_pwm_duty();
    }
}

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles all events generated by Timer2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{

}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles all events generated by Timer3
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles all events generated by Timer4
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
    if(SET == TIM_GetITStatus(TIM4, TIM_IT_Trigger)) {
        uint16_t distanceInTime_us = TIM_GetCounter(TIM4), signal_pin = \
                SENSOR_DIST_FRONT == sensorInUse ? GPIO_Pin_7:GPIO_Pin_6;

        TIM_ClearITPendingBit(TIM4, TIM_IT_Trigger);
        // filter out events of duration < 10us which are volt idles on sensor power off
        if(Bit_RESET == GPIO_ReadInputDataBit(GPIOB, signal_pin) && 10 < distanceInTime_us) {
            distance[sensorInUse] = (340.0f*distanceInTime_us/2.0f)*0.0001f;
            xTaskNotifyFromISR(xTaskGetHandle("Sensors"), (uint32_t){0}, eNoAction, NULL);
        }
    }
}

/*******************************************************************************
* Function Name  : PVD_IRQHandler
* Description    : This function handles events generated by ext. interrupt 
                   line 16 (internally connected to voltage detector)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PVD_IRQHandler(void)
{

    if(SET == EXTI_GetITStatus(EXTI_Line16)) {
        EXTI_ClearITPendingBit(EXTI_Line16);
        FlagStatus PVDO_state = PWR_GetFlagStatus(PWR_FLAG_PVDO);
        vCheckSupplyVoltage(PVDO_state);
    }

}
/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

//#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
//void USB_FS_WKUP_IRQHandler(void)
//#else
//void USBWakeUp_IRQHandler(void)
//#endif
//{
//  EXTI_ClearITPendingBit(EXTI_Line18);
//}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
