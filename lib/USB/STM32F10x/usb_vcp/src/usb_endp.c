/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE] = {0};
volatile uint16_t USB_Rx_Counter = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
    if(1 == USB_Tx_State) {
        if(0 != USB_Tx_length) {
            uint32_t USB_Tx_ptr = USB_Tx_ptr_out, USB_TrTx_length = \
                                USB_Tx_length > VIRTUAL_COM_PORT_DATA_SIZE ?
                                VIRTUAL_COM_PORT_DATA_SIZE:USB_Tx_length;

            USB_Tx_ptr_out += USB_TrTx_length;
            USB_Tx_length -= USB_TrTx_length;

            UserToPMABufferCopy(&USB_Tx_Buffer[USB_Tx_ptr], \
                        ENDP1_TXADDR, (uint16_t)USB_TrTx_length);
            SetEPTxCount(ENDP1, (uint16_t)USB_TrTx_length);
            SetEPTxValid(ENDP1);
        }
        else {
            USB_Tx_State = 0;
        }
    }
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  /* Get the received data buffer and update the counter */
  USB_Rx_Counter = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);

  /* USB data will be immediately processed */
  USB_Receive_Data(USB_Rx_Buffer, USB_Rx_Counter);

  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
}


/*******************************************************************************
* Function Name  : SOF_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
  static uint32_t FrameCount = 0;
  
  if(bDeviceState == CONFIGURED)
  {
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
    {
      /* Reset the frame counter */
      FrameCount = 0;
      
      /* Check the data to be sent through IN pipe */
      Handle_USBAsynchXfer();
    }
  }  
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

