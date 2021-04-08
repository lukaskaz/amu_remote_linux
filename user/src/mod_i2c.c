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
#include <stdbool.h>
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mod_i2c.h"

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

#define BITS_IN_BYTE            8
#define I2C_ACK                 0x00
#define I2C_NACK                0x01
#define I2C_READ(addr)          ((addr)|0x01)
#define I2C_WRITE(addr)         ((addr)&0xFE)
#define I2C_NONPREEPT_TASK_PRIO configMAX_PRIORITIES

static inline void vI2C_set_sda_dir(I2CDirection_t dir);
static inline void vI2C_set_scl(I2C_pinState_t state);
static inline void vI2C_set_sda(I2C_pinState_t state);
static inline I2C_pinState_t xI2C_read_sda(void);
static void vI2C_start_transmission(void);
static void vI2C_stop_transmission(void);
static uint8_t xI2C_write_byte(uint8_t);
static uint8_t xI2C_read_byte(uint8_t);


static inline void supp_ticks_delay(volatile register uint16_t ticks)
{
    while(ticks--);
}

static inline void vI2C_set_sda_dir(I2CDirection_t dir)
{
    static const uint32_t regpos = (1<<GPIO_Pin_1), regmask = ~(((uint32_t)0x0F)<<regpos);
    uint32_t mode = I2C_DIR_OUTPUT == dir ? GPIO_Mode_Out_PP:GPIO_Mode_IPD,
             currentmode = mode & (uint32_t)0x0F, tmpreg = GPIOA->CRL & regmask;

    if(GPIO_Mode_Out_PP == mode) currentmode |= GPIO_Speed_50MHz;
    else  GPIOA->BRR = GPIO_Pin_1;
    GPIOA->CRL = tmpreg | (currentmode<<regpos);
}

static inline void vI2C_set_scl(I2C_pinState_t state)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_0, I2C_PIN_LOW == state ? Bit_RESET:Bit_SET);
}

static inline void vI2C_set_sda(I2C_pinState_t state)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, I2C_PIN_LOW == state ? Bit_RESET:Bit_SET);
}

static inline I2C_pinState_t xI2C_read_sda(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET ? \
                                            I2C_PIN_LOW:I2C_PIN_HIGH;
}

static UBaseType_t xI2C_handle_task_prio(UBaseType_t* prio_curr)
{
    static UBaseType_t task_prio = 0;
    if(NULL != prio_curr) task_prio = *prio_curr;
    return task_prio;
}

static void vI2C_block_task_preemption(void)
{
    UBaseType_t task_prio = uxTaskPriorityGet(NULL);
    xI2C_handle_task_prio(&task_prio);
    vTaskPrioritySet(NULL, I2C_NONPREEPT_TASK_PRIO);
}

static void vI2C_restore_task_preemption(void)
{
    UBaseType_t task_prio = xI2C_handle_task_prio(NULL);
    vTaskPrioritySet(NULL, task_prio);
}

/***********************************************************************
 * Name: vI2C_start_transmission()
 * Note: Start I2C Bus, SDA change to low when SCL is hight
 * Para: None
 * Return : None
************************************************************************/
static void vI2C_restart_transmission(void)
{
    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_sda(I2C_PIN_HIGH);
    vI2C_set_scl(I2C_PIN_HIGH);
    vI2C_set_sda(I2C_PIN_LOW);
    vI2C_set_scl(I2C_PIN_LOW);
}

static void vI2C_start_transmission(void)
{
    vI2C_block_task_preemption();
    vI2C_restart_transmission();
}

/***********************************************************************
 * Name: vI2C_stop_transmission()
 * Note: Stop I2C Bus, SCL change to hight when SDA is hight
 * Para: None
 * Return : None
************************************************************************/
static void vI2C_stop_transmission(void)
{
    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_scl(I2C_PIN_LOW);
    vI2C_set_sda(I2C_PIN_LOW);
    vI2C_set_scl(I2C_PIN_HIGH);
    vI2C_set_sda(I2C_PIN_HIGH);
    vI2C_restore_task_preemption();
}

/***********************************************************************
 * Name: xI2C_write_byte(dat)
 * Note: Write 8bite data and get ack;
 * Para: dat -> the data which will be send out
 * Return : ack -> ack signal
************************************************************************/
static uint8_t xI2C_write_byte(uint8_t byte)
{
    static const uint16_t write_delay = 5;
    uint8_t status = 1;

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);

    for(uint16_t i=0, bit = 0x80; i < BITS_IN_BYTE; i++) {
        vI2C_set_sda(byte&(bit>>i) ? I2C_PIN_HIGH:I2C_PIN_LOW);
        vI2C_set_scl(I2C_PIN_HIGH);
        supp_ticks_delay(write_delay);
        vI2C_set_scl(I2C_PIN_LOW);
        supp_ticks_delay(write_delay);
    }

    vI2C_set_sda_dir(I2C_DIR_INPUT);
    vI2C_set_scl(I2C_PIN_HIGH);
    supp_ticks_delay(write_delay);
    status = (uint8_t)xI2C_read_sda();
    vI2C_set_scl(I2C_PIN_LOW);

    return status;
}

/***********************************************************************
 * Name: xI2C_read_byte
 * Note: Read 8bite data and Send  ack;
 * Para: ack=0 -> Set ack, ack=1 -> Set noack
 * Return : read data
************************************************************************/
static uint8_t xI2C_read_byte(uint8_t status)
{
    static const uint16_t read_delay = 5;
    uint8_t byte = 0x00;

    vI2C_set_sda_dir(I2C_DIR_INPUT);
    for(uint16_t i=0, bit = 0x80; i < BITS_IN_BYTE; i++) {
        vI2C_set_scl(I2C_PIN_HIGH);
        supp_ticks_delay(read_delay);
        byte = I2C_PIN_HIGH == xI2C_read_sda() ? byte|(bit>>i):byte;
        vI2C_set_scl(I2C_PIN_LOW);
        supp_ticks_delay(read_delay);
    }

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_sda(status);
    vI2C_set_scl(I2C_PIN_HIGH);
    supp_ticks_delay(read_delay);
    vI2C_set_scl(I2C_PIN_LOW);

    return byte;
}

/***********************************************************************
 * Name: xI2C_read_byte
 * Note: Read 8bite data and Send  ack;
 * Para: ack=0 -> Set ack, ack=1 -> Set noack
 * Return : read data
************************************************************************/
static uint8_t xI2C_read_byte_slow(uint8_t status)
{
    static const uint16_t read_delay = 50;
    uint8_t byte = 0x00;

    vI2C_set_sda_dir(I2C_DIR_INPUT);
    for(uint16_t i=0, bit = 0x80; i < BITS_IN_BYTE; i++) {
        vI2C_set_scl(I2C_PIN_HIGH);
        supp_ticks_delay(read_delay);
        byte = I2C_PIN_HIGH == xI2C_read_sda() ? byte|(bit>>i):byte;
        vI2C_set_scl(I2C_PIN_LOW);
        supp_ticks_delay(read_delay);
    }

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_sda(status);
    vI2C_set_scl(I2C_PIN_HIGH);
    supp_ticks_delay(read_delay);
    vI2C_set_scl(I2C_PIN_LOW);

    return byte;
}

/***********************************************************************
 * Name: xI2C_write_sequence
 * Note: Write data to device
 * Para: device -> device address, buf->subaddress + data
 * Return : how many bytes have been write
************************************************************************/
uint8_t xI2C_write_sequence(I2cParams_t* params)
{
    const uint8_t* data = params->data;
    uint16_t written = 0, size = params->dataSize;

    vI2C_start_transmission();
    if(I2C_ACK == xI2C_write_byte(I2C_WRITE(params->devAddr))) {   //shift device addres left by 1 and clear last bit (write op)
        if(I2C_ACK == xI2C_write_byte(params->ctrlReg)) {  //shift device addres left by 1 and clear last bit (write op)
            for(uint8_t res = I2C_ACK; res == I2C_ACK && written < size; written++) {
                res = xI2C_write_byte(*data++);
            }
        }
    }
    vI2C_stop_transmission();

    return written;
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
    uint8_t* data = params->data;
    uint16_t read = 0, size_to_ack = params->dataSize - 1;

    vI2C_start_transmission();
    if(I2C_ACK == xI2C_write_byte(I2C_WRITE(params->devAddr))) {  //shift device addres left by 1 and clear last bit (write op)
        if(xI2C_write_byte(params->ctrlReg) == 0) {
            vI2C_restart_transmission();
            xI2C_write_byte(I2C_READ(params->devAddr));   //shift device addres left by 1 and add read bit

            for(read=0; read < size_to_ack; read++) {
                *data++ = xI2C_read_byte(I2C_ACK);
            }
            *data = xI2C_read_byte(I2C_NACK);
            read++;
        }
    }
    vI2C_stop_transmission();

    return read;
}

/***********************************************************************
 * Name: xI2C_read_sequence
 * Note: Read data from device
 * Para: device->device address, subaddr->subaddress, acnt->subaddress lengh
 *       buf->read out data space, bcnt->read out data lengh
 * Return : write subaddress lengh
************************************************************************/
uint8_t xI2C_read_sequence_slow(I2cParams_t* params)
{
    uint8_t* data = params->data;
    uint16_t read = 0, size_to_ack = params->dataSize - 1;

    vI2C_start_transmission();
    if(I2C_ACK == xI2C_write_byte(I2C_WRITE(params->devAddr))) {  //shift device addres left by 1 and clear last bit (write op)
        if(xI2C_write_byte(params->ctrlReg) == 0) {
            vI2C_restart_transmission();
            xI2C_write_byte(I2C_READ(params->devAddr));   //shift device addres left by 1 and add read bit

            for(read=0; read < size_to_ack; read++) {
                *data++ = xI2C_read_byte_slow(I2C_ACK);
            }
            *data = xI2C_read_byte_slow(I2C_NACK);
            read++;
        }
    }
    vI2C_stop_transmission();

    return read;
}

void vI2C_configuration(void)
{
    //SCL signal and SDA signal
    static GPIO_InitTypeDef GPIO_InitStructure =
    {
        .GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_Out_PP
    };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    vI2C_set_sda(I2C_PIN_HIGH);
    vI2C_set_scl(I2C_PIN_HIGH);
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
