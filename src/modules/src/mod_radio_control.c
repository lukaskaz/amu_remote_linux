/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_radio_control.c
** Descriptions:            Interface for RF radio operations
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

#include "mod_radio_control.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include "mod_drive.h"
#include "mod_lighting.h"
#include "mod_sound_signal.h"
#include "mod_lcd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define RADIO_CTRL_MS_INTERVAL_NONE       0
#define RADIO_CTRL_MS_INTERVAL_CONSOLE    100U
#define RADIO_CTRL_MS_INTERVAL_JOYSTICK   50U
#define RADIO_CTRL_MS_INTERVAL_BLUETOOTH  150U
#define RADIO_OP_QUEUE_SIZE               4U

#define RADIO_BYTES_FOR_CS                (RADIO_FRAME_SIZE - 1U)

typedef enum {
    RADIO_OP_NONE = 0,
    RADIO_OP_DRIVE,
    RADIO_OP_LIGHTING,
    RADIO_OP_SOUND_SIG,
    RADIO_OP_HEARTBEAT,
} RadioOperation_t;

typedef struct {
    RadioOperation_t operation[RADIO_OP_QUEUE_SIZE];
    uint8_t operationPos;
} radioOperationQueue_t;

void vRadio_configuration(void);
static bool isOperationWaiting(const radioOperationQueue_t *queue, const RadioOperation_t op);
static bool releaseOperation(radioOperationQueue_t *queue, const RadioOperation_t op);
static void addOperation(radioOperationQueue_t *queue, const RadioOperation_t op);
static RadioOperation_t getOperation(radioOperationQueue_t *queue);
static uint8_t radio_data_checksum_calculate(const radioData_t *const data, uint8_t mode);

xSemaphoreHandle xSemaphRadioPacketReady = NULL;
bool startRadioRefresh = false;
bool startRadioSendUpdate = false;
radioData_t radioData = {0};
const uint16_t radioIntervalDelay[4] = { RADIO_CTRL_MS_INTERVAL_NONE, RADIO_CTRL_MS_INTERVAL_CONSOLE,
                                         RADIO_CTRL_MS_INTERVAL_JOYSTICK, RADIO_CTRL_MS_INTERVAL_BLUETOOTH };

static bool isOperationWaiting(const radioOperationQueue_t *queue, const RadioOperation_t op)
{
    bool fresult = false;
    uint8_t i = 0;
    
    for(i=0; i<queue->operationPos; i++) {
        if(queue->operation[i] == op) {
            fresult = true;
            break;
        }
    }

    return fresult;
}

static bool releaseOperation(radioOperationQueue_t *queue, const RadioOperation_t op)
{
    bool fresult = false;
    uint8_t i = 0;
    
    for(i=0; i<queue->operationPos; i++) {
        if(queue->operation[i] == op) {
            queue->operationPos--;
            for(; i<queue->operationPos; i++) {
                queue->operation[i] = queue->operation[i+1];
            }
            queue->operation[i] = RADIO_OP_NONE;
            fresult = true;
            break;
        }
    }

    return fresult;
}

static void addOperation(radioOperationQueue_t *queue, const RadioOperation_t op)
{
    if(op != RADIO_OP_NONE && isOperationWaiting(queue, op) != true) {
        queue->operation[queue->operationPos] = op;
        queue->operationPos++;
        if(queue->operationPos == RADIO_OP_QUEUE_SIZE) {
            // when buffer overflows only the last cell can be updated
            // other cells are being kept until removed intentionally by software
            queue->operationPos--;
        }
    }
}

static RadioOperation_t getOperation(radioOperationQueue_t *queue)
{
    RadioOperation_t fresult = RADIO_OP_NONE;
    uint8_t i = 0;
    
    if(queue->operationPos != 0) {
        fresult = queue->operation[0];
        queue->operationPos--;
        
        for(i=0; i<queue->operationPos; i++) {
            queue->operation[i] = queue->operation[i+1];
        }
        queue->operation[i] = RADIO_OP_NONE;
    }

    return fresult;
}

void vRadio_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

    //configure GPIO pins for USART1
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //configure USART
    USART_InitStructure.USART_BaudRate   = 125000;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_WakeUpConfig(USART1, USART_WakeUp_AddressMark);
    USART_SetAddress(USART1, 0x05);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    USART_ClearFlag(USART1,USART_FLAG_RXNE);
    USART_ClearFlag(USART1,USART_FLAG_TC);
    USART_Cmd(USART1, ENABLE);
}

void vRadioTimerCallback(xTimerHandle pxTimer)
{
    uint32_t timerId = (uint32_t)pvTimerGetTimerID(pxTimer);
    //printf("[%s] got %d\n", __func__, timerId);
    switch(timerId) {
        case 1:
            startRadioRefresh = true;
            break;
        case 2:
            startRadioSendUpdate = true;
            break;
        default:
            break;
    }
    
}


void vRadio_Control(void *pvArg)
{
    xTimerHandle xRadioRefreshTimer;
    xTimerHandle xRadioSendUpdateTimer;
    radioOperationQueue_t radioOperationQueue = {RADIO_OP_NONE};
    radioController_t controller = RADIO_CTRL_CONSOLE;

    vRadio_configuration();

    xRadioRefreshTimer = xTimerCreate((signed char*)"Radio refresh tmr", radioIntervalDelay[controller], pdFALSE, (void *)1, vRadioTimerCallback);
    xRadioSendUpdateTimer = xTimerCreate((signed char*)"Radio send status tmr", 100, pdTRUE, (void *)2, vRadioTimerCallback);
    //xTimerStart(xRadioSendUpdateTimer, 0);

    while(1) {
        lcdControlData_t lcdData = {0};

        if(xSemaphoreTake(xSemaphRadioPacketReady, 10) == pdTRUE) {
            xTimerStart(xRadioRefreshTimer, 0);
            

            printf("Got radio command: %d\n\r", radioData.radioRxMsgReady.payload.operation);
            if(radioData.radioRxMsgReady.payload.common.controller != controller) {
                controller = (radioController_t)radioData.radioRxMsgReady.payload.common.controller;
                xTimerChangePeriod(xRadioRefreshTimer, radioIntervalDelay[controller], 0);
            }

            if(radioData.radioRxMsgReady.payload.operation == RADIO_OP_DRIVE) {
                driveControlData_t driveControlData = {0};

                driveControlData.direction  = radioData.radioRxMsgReady.payload.driveData.direction;
                driveControlData.speed_0    = radioData.radioRxMsgReady.payload.driveData.speed_0;
                driveControlData.speed_1    = radioData.radioRxMsgReady.payload.driveData.speed_1;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                
                if(isOperationWaiting(&radioOperationQueue, RADIO_OP_DRIVE) == false) {
                    lcdData.operation = LCD_OP_DRIVE;
                    lcdData.state = LCD_MV_IN_MOTION;
                }
                if(isOperationWaiting(&radioOperationQueue, RADIO_OP_SOUND_SIG) == true) {
                    releaseOperation(&radioOperationQueue, RADIO_OP_SOUND_SIG);
                    vSound_Signal_RF_Control(SOUND_RF_NONE);

                    lcdData.operation = LCD_OP_DRIVE;
                    lcdData.state = LCD_MV_IN_MOTION;
                }
            }
            else if(radioData.radioRxMsgReady.payload.operation == RADIO_OP_LIGHTING) {
                vLighting_RF_Control(radioData.radioRxMsgReady.payload.lightingData.lightingType,
                                        radioData.radioRxMsgReady.payload.lightingData.lightingState);
                xTimerStart(xRadioSendUpdateTimer, 0);
                startRadioSendUpdate = true;
            }
            else if(radioData.radioRxMsgReady.payload.operation == RADIO_OP_SOUND_SIG) {
                if(isOperationWaiting(&radioOperationQueue, RADIO_OP_SOUND_SIG) == false) {
                    vSound_Signal_RF_Control(radioData.radioRxMsgReady.payload.soundSignalData.soundSignalStatus);
                    lcdData.operation = LCD_OP_SOUND_SIG;
                    lcdData.state = LCD_SOUND_ON;
                }
            }
            else if(radioData.radioRxMsgReady.payload.operation == RADIO_OP_HEARTBEAT) {
                // heartbeat signal for diagnosis purpuses on controller side, do nothing
            }
            else {
                // unsupported case, do nothing
            }
            
            addOperation(&radioOperationQueue, (RadioOperation_t)radioData.radioRxMsgReady.payload.operation);
            //lastOperation = radioData.operations.payload.function;
            //printf("Sig: %d, %d\r\n", radioData.operations.payload.function, radioData.operations.payload.soundSignalData.soundSignalStatus);
        }
        else if(startRadioRefresh == true) {
            startRadioRefresh = false;
            
            //printf("off\r\n");
            while(1) {
                RadioOperation_t operation = getOperation(&radioOperationQueue);
                if(operation == RADIO_CTRL_NONE) {
                    break;
                }
                else {
                    if(operation == RADIO_OP_DRIVE) {
                        driveControlData_t driveControlData = {0};

                        driveControlData.direction = DRIVE_OP_STOPPED;
                        driveControlData.speed_0   = 0;
                        driveControlData.speed_1   = 0;
                        xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                        
                        lcdData.operation = LCD_OP_DRIVE;
                        lcdData.state = LCD_MV_STOPPED;
                    }
                    else if(operation == RADIO_OP_SOUND_SIG) {
                        // there is no sound signal package received so suppress horn
                        vSound_Signal_RF_Control(SOUND_RF_NONE);
                        lcdData.operation = LCD_OP_SOUND_SIG;
                        lcdData.state = LCD_SOUND_OFF;
                    }
                    else {
                        // unsupported operation, do nothing
                    }
                }
            }
        }

        if(startRadioSendUpdate == true) {
            startRadioSendUpdate = false;
            
            //printf("sending update!!!!\r\n");

            int i = 0;
            for(i=0; i<RADIO_FRAME_SIZE; i++) {
                radioData.frame.uartTxFrame.buffer[i] = 0x00;
            }

            radioData.frame.uartTxFrame.txMsg.function          = UART_FUNC_MSG_SEND;
            radioData.frame.uartTxFrame.txMsg.senderAddr        = RADIO_UART_ADDRESS;
            radioData.frame.uartTxFrame.txMsg.radioTargetAddr   = RADIO_ADDR_NODE_1;
            radioData.frame.uartTxFrame.txMsg.payloadBytesNb    = RADIO_PAYLOAD_SIZE;
            
            uint8_t state = 0;
            vGet_Lighting_State(&state);
            radioData.frame.uartTxFrame.txMsg.payload.lightingData.operation = RADIO_OP_LIGHTING;
            radioData.frame.uartTxFrame.txMsg.payload.lightingData.controller = 0;
            radioData.frame.uartTxFrame.txMsg.payload.lightingData.lightingType = state;
            radioData.frame.uartTxFrame.txMsg.payload.lightingData.lightingState = 1;
            
            radioData.frame.uartTxFrame.txMsg.checksum = radio_data_checksum_calculate(&radioData, 2);
            radioData.txState = UART_FRAME_ST_TRANSMITTING;

            USART_ClearFlag(USART1, USART_FLAG_TXE);
            USART_SendData(USART1, RADIO_UART_SLAVE_ADDRESS);
            //printf("data_tx: %d", RADIO_UART_SLAVE_ADDRESS);
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

            for(i=0; i<RADIO_FRAME_SIZE; i++) {
                USART_ClearFlag(USART1, USART_FLAG_TXE);
                USART_SendData(USART1, radioData.frame.uartTxFrame.buffer[i]);
                //printf(", %d", radioData.frame.buffer[i]);
                while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            }

            USART_SendData(USART1, RADIO_UART_END_SIGNATURE);
            uart_data_sent = false;
            while(uart_data_sent == false);
            //printf(", %d\n", RADIO_UART_END_SIGNATURE);
            //printf("data_tx: %d", (0x100|0x0A));
            //for(i=0; i<RADIO_FRAME_SIZE; i++) {
                //printf(", %d",radioData.frame.uartTxFrame.buffer[i]);
            //}
            //printf(", %d\n", 0x100|0xFF);

            int radioRespWaitTimeoutMs = 80;
            while(radioData.txState != UART_FRAME_ST_READY && radioRespWaitTimeoutMs != 0) {
                vTaskDelay(1);
                radioRespWaitTimeoutMs--;
            }

            printf("resp: %d : %d\n", radioData.radioRxRespReady.function, radioData.radioRxRespReady.response);
            
//            xTimerReset(xRadioSendUpdateTimer, 0);
//            startRadioSendUpdate = false;
            //vTaskDelay(5);
        }
        else {
            // no action defined
        }

        if(lcdData.operation != LCD_OP_NONE) {
            //xQueueSend(xQueueLcdControl, (void *)&lcdData, 0);
        }
    }
}

bool is_radio_data_checksum_correct(const radioData_t *const data)
{
    if( data->frame.uartRxFrame.rxMsg.checksum == radio_data_checksum_calculate(data, 1)) {
        return true;
    }

    return false;
}

static uint8_t radio_data_checksum_calculate(const radioData_t *const data, uint8_t mode)
{
    uint16_t checksum = 0, i = 0;
    const uint8_t* buffer = NULL;
    
    if(mode == 1) {
        buffer = data->frame.uartRxFrame.buffer;
    }
    else {
        buffer = data->frame.uartTxFrame.buffer;
    }

    for(i=0; i<RADIO_BYTES_FOR_CS; i++) {
        checksum += buffer[i];
    }

    checksum = (uint8_t)((checksum^0xFF) + 1);
    return checksum;
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
