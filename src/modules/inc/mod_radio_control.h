/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_RADIO_CONTROL_H
#define MOD_RADIO_CONTROL_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define RADIO_UART_ADDRESS              5U
#define RADIO_UART_MASTER_ADDRESS       (0x100|RADIO_UART_ADDRESS)
#define RADIO_UART_SLAVE_ADDRESS        (0x100|0x0A)
#define RADIO_UART_END_SIGNATURE    (0x100|0xFF)

#define RADIO_FRAME_SIZE              25U
#define RADIO_RESP_FRAME_SIZE         2U
#define RADIO_PAYLOAD_SIZE            20U

typedef enum
{
    UART_FUNC_UNDEFINED = 0,
    UART_FUNC_MSG_SEND,
    UART_FUNC_MSG_RECEIVE,
    UART_FUNC_RESPONSE,
}uartFunction_t;

typedef enum {
    UART_FRAME_ST_UNDEFINED = 0,
    UART_FRAME_ST_TRANSMITTING,
    UART_FRAME_ST_RECEIVING,
    UART_FRAME_ST_READY,
    UART_FRAME_ST_UNSUPPORTED
} uartFrameState_t;

typedef enum
{
    RADIO_ADDR_BASE = 0,
    RADIO_ADDR_NODE_1,
    RADIO_ADDR_NODE_MAX,
}radioAddress_t;

typedef enum
{
    RADIO_RES_UNDEFINED = 0,
    RADIO_RES_VERIFICATION_ERROR,
    RADIO_RES_TRANSFER_INTERRUPTED,
    RADIO_RES_TRANSFER_ERROR,
    RADIO_RES_TRANSFER_COMPLETE,
    RADIO_RES_UNSUPPORTED,
}radioResp_t;

typedef struct{
    uint8_t function;
    uint8_t senderAddr;
    uint8_t radioTargetAddr;
    uint8_t payloadBytesNb;
    
    union {
        uint8_t operation;
        struct {
            uint8_t data[RADIO_PAYLOAD_SIZE];
        } plainData;
        struct {
            uint8_t operation;
            uint8_t controller;
        } common;
        struct {
            uint8_t operation;
            uint8_t controller;
            uint8_t direction;
            uint8_t speed_0;
            uint8_t speed_1;
        } driveData;
        struct {
            uint8_t operation;
            uint8_t controller;
            uint8_t lightingType;
            uint8_t lightingState;
        } lightingData;
        struct {
            uint8_t operation;
            uint8_t controller;
            uint8_t soundSignalStatus;
        } soundSignalData;
    } payload;
    uint8_t checksum;
} radioRxMsg_t;

typedef struct {
    uint8_t function;
    radioResp_t response;
} radioRxResp_t;

typedef struct {
    uint8_t function;
    uint8_t senderAddr;
    uint8_t radioTargetAddr;
    uint8_t payloadBytesNb;
    
    union {
        uint8_t operation;
        struct {
            uint8_t data[RADIO_PAYLOAD_SIZE];
        } plainData;
        struct {
            uint8_t operation;
            uint8_t controller;
        } common;
        struct {
            uint8_t operation;
            uint8_t controller;
            uint8_t lightingType;
            uint8_t lightingState;
        } lightingData;
    } payload;
    uint8_t checksum;
} radioTxMsg_t;

typedef struct {
    uartFrameState_t rxState;
    uartFrameState_t txState;
    
    struct {
        union {
            uint8_t buffer[RADIO_FRAME_SIZE];
            radioRxMsg_t rxMsg;
            radioRxResp_t rxResp;
        } uartRxFrame;
        
        union {
            uint8_t buffer[RADIO_FRAME_SIZE];
            radioTxMsg_t txMsg;
        } uartTxFrame;
        
    }frame;

    radioRxMsg_t radioRxMsgReady;
    radioRxResp_t radioRxRespReady;
} radioData_t;

typedef enum {
    RADIO_CTRL_NONE = 0,
    RADIO_CTRL_CONSOLE,
    RADIO_CTRL_JOYSTICK,
    RADIO_CTRL_BLUETOOTH,
} radioController_t;


extern void vRadio_Control(void *pvArg);
extern bool is_radio_data_checksum_correct(const radioData_t *const data);

extern radioData_t radioData;
extern xSemaphoreHandle xSemaphRadioPacketReady;
extern const uint16_t radioIntervalDelay[];

#endif
