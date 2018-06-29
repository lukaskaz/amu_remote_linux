/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_i2c.c
** Descriptions:            i2c protocol interface implemented with software only
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
#include "mod_i2c.h"
#include "task.h"
#include "stm32f10x.h"

#define I2C_TIME_DELAY_TICKS    20U

typedef enum {
    I2C_DIR_OUTPUT = 0,
    I2C_DIR_INPUT,
} I2CDirection_t;

typedef enum {
    I2C_OP_READ_SEQ = 0,
    I2C_OP_WRITE_SEQ
} I2C_operation_t;

typedef enum {
    I2C_PIN_LOW = 0,
    I2C_PIN_HIGH,
    I2C_PIN_UNDEF = 0xFF
} I2C_pinState_t;

static inline void vI2C_ticks_delay(I2C_operation_t op);
static inline void vI2C_set_sda_dir(I2CDirection_t dir);
static inline void vI2C_set_scl(I2C_pinState_t state);
static inline void vI2C_set_sda(I2C_pinState_t state);
static inline I2C_pinState_t xI2C_read_sda_state(void);
static void vI2C_start_transmission(I2C_operation_t op);
static void vI2C_stop_transmission(I2C_operation_t op);
static uint8_t xI2C_write_byte(I2C_operation_t op, uint8_t dat);
static uint8_t xI2C_read_byte(I2C_operation_t op, uint8_t ack);

xSemaphoreHandle xSemaphI2CLcdInitDone = NULL;


static inline void vI2C_ticks_delay(I2C_operation_t op)
{
    volatile register uint32_t delay = 0;

    if(op == I2C_OP_WRITE_SEQ) {
        delay = 1;
        // just symbolic delay here, perform full speed transfer
    }
    else if(op == I2C_OP_READ_SEQ) {
        delay = I2C_TIME_DELAY_TICKS;
    }
    else {
        // not supported case, abort
        return;
    }

    while(delay--);
}

static inline void vI2C_set_sda_dir(I2CDirection_t dir)
{
    static GPIO_InitTypeDef GPIO_InitStructure =
    {
        .GPIO_Pin = GPIO_Pin_1,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_IPD
    };

    switch(dir) {
        case I2C_DIR_INPUT:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
            break;
        case I2C_DIR_OUTPUT:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
            break;
        default:
            break;
    }

    GPIO_ChgSinglePinMode(GPIOA, &GPIO_InitStructure);
}

static inline void vI2C_set_scl(I2C_pinState_t state)
{
    switch(state) {
        case I2C_PIN_LOW:
            GPIO_ResetBits(GPIOA, GPIO_Pin_0);
            break;
        case I2C_PIN_HIGH:
            GPIO_SetBits(GPIOA, GPIO_Pin_0);
            break;
        default:
            break;
    }
}

static inline void vI2C_set_sda(I2C_pinState_t state)
{
    switch(state) {
        case I2C_PIN_LOW:
            GPIO_ResetBits(GPIOA, GPIO_Pin_1);
            break;
        case I2C_PIN_HIGH:
            GPIO_SetBits(GPIOA, GPIO_Pin_1);
            break;
        default:
            break;
    }
}

static inline I2C_pinState_t xI2C_read_sda_state(void)
{
    register I2C_pinState_t dat = I2C_PIN_UNDEF;

    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_SET) {
        dat = I2C_PIN_HIGH;
    }
    else {
        dat = I2C_PIN_LOW;
    }

    return dat;
}

/***********************************************************************
 * Name: vI2C_start_transmission()
 * Note: Start I2C Bus, SDA change to low when SCL is hight
 * Para: None
 * Return : None
************************************************************************/
static void vI2C_start_transmission(I2C_operation_t op)
{

    if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskSuspendAll();
    }

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);

    vI2C_ticks_delay(op);
    vI2C_set_sda(I2C_PIN_HIGH);
    vI2C_set_scl(I2C_PIN_HIGH);
    vI2C_ticks_delay(op);
    vI2C_set_sda(I2C_PIN_LOW);
    vI2C_ticks_delay(op);
    vI2C_set_scl(I2C_PIN_LOW);
    vI2C_ticks_delay(op);
}

/***********************************************************************
 * Name: vI2C_stop_transmission()
 * Note: Stop I2C Bus, SCL change to hight when SDA is hight
 * Para: None
 * Return : None
************************************************************************/
static void vI2C_stop_transmission(I2C_operation_t op)
{
    vI2C_set_sda_dir(I2C_DIR_OUTPUT);

    vI2C_set_scl(I2C_PIN_LOW);
    vI2C_set_sda(I2C_PIN_LOW);
    vI2C_ticks_delay(op);
    vI2C_set_scl(I2C_PIN_HIGH);
    vI2C_ticks_delay(op);
    vI2C_set_sda(I2C_PIN_HIGH);
    vI2C_ticks_delay(op);

    if(xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) {
        xTaskResumeAll();
    }
}

/***********************************************************************
 * Name: xI2C_write_byte(dat)
 * Note: Write 8bite data and get ack;
 * Para: dat -> the data which will be send out
 * Return : ack -> ack signal
************************************************************************/
static uint8_t xI2C_write_byte(I2C_operation_t op, uint8_t dat)
{
    uint8_t i = 0, ack = 1;

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);

    for(i=0; i<8; i++)
    {
        vI2C_ticks_delay(op);

        if(dat & 0x80) {
            vI2C_set_sda(I2C_PIN_HIGH);
        }
        else {
            vI2C_set_sda(I2C_PIN_LOW);
        }
        dat<<=1;

        vI2C_ticks_delay(op);
        vI2C_set_scl(I2C_PIN_HIGH);
        vI2C_ticks_delay(op);
        vI2C_ticks_delay(op);
        vI2C_set_scl(I2C_PIN_LOW);
    }

    vI2C_ticks_delay(op);
    vI2C_ticks_delay(op);
    vI2C_set_sda_dir(I2C_DIR_INPUT);
    vI2C_set_scl(I2C_PIN_HIGH);
    vI2C_ticks_delay(op);
    vI2C_ticks_delay(op);
    ack = (uint8_t)xI2C_read_sda_state();
    vI2C_set_scl(I2C_PIN_LOW);

    return ack;
}

/***********************************************************************
 * Name: xI2C_read_byte
 * Note: Read 8bite data and Send  ack;
 * Para: ack=0 -> Set ack, ack=1 -> Set noack
 * Return : read data
************************************************************************/
static uint8_t xI2C_read_byte(I2C_operation_t op, uint8_t ack)
{
    uint8_t i = 0, dat = 0xFF;

    vI2C_set_sda_dir(I2C_DIR_INPUT);
    for(i=0; i<8; i++)
    {
        vI2C_ticks_delay(op);
        vI2C_ticks_delay(op);
        vI2C_set_scl(I2C_PIN_HIGH);
        vI2C_ticks_delay(op);

        dat<<=1;
        if(xI2C_read_sda_state() == I2C_PIN_HIGH) {
            dat |= 1;
        }

        vI2C_ticks_delay(op);
        vI2C_set_scl(I2C_PIN_LOW);
    }

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_sda(ack);    // ack = 0; ask, ack = 1,stop
    vI2C_ticks_delay(op);
    vI2C_ticks_delay(op);
    vI2C_set_scl(I2C_PIN_HIGH);
    vI2C_ticks_delay(op);
    vI2C_ticks_delay(op);
    vI2C_set_scl(I2C_PIN_LOW);

    return dat;
}

/***********************************************************************
 * Name: xI2C_write_sequence
 * Note: Write data to device
 * Para: device -> device address, buf->subaddress + data
 * Return : how many bytes have been write
************************************************************************/
//uint8_t xI2C_write_sequence(uint8_t device, uint8_t *buf, uint8_t cnt)
//{
//    uint8_t i = 0;
//    static const I2C_operation_t op = I2C_OP_WRITE_SEQ;
//
//    vI2C_start_transmission(op);
//    if(xI2C_write_byte(op, (device<<1)&(0xFE)) == 0)   //shift device addres left by 1 and clear last bit (write op)
//    {
//        for(i=0; i<cnt; i++) {
//            if(xI2C_write_byte(op, buf[i]) == 1) {
//                break;
//            }
//        }
//    }
//    else {
//        // nack for given device address byte, end i2c communication
//    }
//    vI2C_stop_transmission(op);
//
//    return i;
//}


uint8_t xI2C_write_sequence1(I2cParams_t* params)
{
    uint8_t i = 0;
    static const I2C_operation_t op = I2C_OP_WRITE_SEQ;

    vI2C_start_transmission(op);
    if(xI2C_write_byte(op, (params->devAddr)&(0xFE)) == 0)   //shift device addres left by 1 and clear last bit (write op)
    {
        if(xI2C_write_byte(op, params->ctrlReg) == 0)   //shift device addres left by 1 and clear last bit (write op)
        {
            for(i=0; i < params->dataSize; i++) {
                if(xI2C_write_byte(op, params->data[i]) == 1) {
                    break;
                }
            }
        }
    }
    else {
        // nack for given device address byte, end i2c communication
    }
    vI2C_stop_transmission(op);

    return i;
}

/***********************************************************************
 * Name: xI2C_read_sequence
 * Note: Read data from device
 * Para: device->device address, subaddr->subaddress, acnt->subaddress lengh
 *       buf->read out data space, bcnt->read out data lengh
 * Return : write subaddress lengh
************************************************************************/
uint8_t xI2C_read_sequence(I2cParams_t* params)
{
    static const I2C_operation_t op = I2C_OP_READ_SEQ;
    uint8_t i = 0;

    vI2C_start_transmission(op);
    if(xI2C_write_byte(op, (params->devAddr)&(0xFE)) == 0)   //shift device addres left by 1 and clear last bit (write op)
    {
        if(xI2C_write_byte(op, params->ctrlReg) == 0) {
            vI2C_start_transmission(op);
            xI2C_write_byte(op, (params->devAddr)|(0x01));   //shift device addres left by 1 and add read bit

            for(i=0; i < (params->dataSize-1); i++) {
                params->data[i] = xI2C_read_byte(op, 0); // read & send ACK
            }
            params->data[i] = xI2C_read_byte(op, 1); // read & send noack
        }
        else {
            // fail to write control register, abort
        }
    }
    else {
        // nack for given device address byte, end i2c communication
    }
    vI2C_stop_transmission(op);

    return i;
}

void vI2C_configuration(void)
{
    static GPIO_InitTypeDef GPIO_InitStructure =
    {
        .GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_Out_PP
    };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //SCL signal and SDA signal
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //SDA signal
    //GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    //GPIO_Init(GPIOA, &GPIO_InitStructure);

    //vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_sda(I2C_PIN_HIGH);
    vI2C_set_scl(I2C_PIN_HIGH);
    vI2C_ticks_delay(I2C_OP_READ_SEQ);
    vI2C_ticks_delay(I2C_OP_READ_SEQ);
}




/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
