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
#include "stm32f10x.h"

#define IIC_TIME_DELAY_TICKS    100U

typedef enum {
    I2C_DIR_OUTPUT = 0,
    I2C_DIR_INPUT,
} I2CDirection_t;

static void vI2C_ticks_delay(uint32_t i);
static void vI2C_set_sda_dir(I2CDirection_t dir);
static void vI2C_set_scl(uint8_t state);
static void vI2C_set_sda(uint8_t state);
static uint8_t xI2C_read_sda_state(void);
static void vI2C_start_transmission(void);
static void vI2C_stop_transmission(void);
static uint8_t xI2C_write_byte(uint8_t dat);
static uint8_t xI2C_read_byte(uint8_t ack);

xSemaphoreHandle xRecMutexI2CSequence = NULL;
xSemaphoreHandle xSemaphI2CLcdInitDone = NULL;

static void vI2C_ticks_delay(uint32_t i)
{
    while(i--);
}

static void vI2C_set_sda_dir(I2CDirection_t dir)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if(dir == I2C_DIR_INPUT) {
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 
    }
    else if(dir == I2C_DIR_OUTPUT) {
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    }
    else {
        // should not occur, do nothing
    }
    
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void vI2C_set_scl(uint8_t state)
{
    if(state) {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
    }
    else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    }
}

static void vI2C_set_sda(uint8_t state)
{
    if(state) {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
    }
    else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    }
}

static uint8_t xI2C_read_sda_state(void)
{
    uint8_t dat;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_SET) {
        dat = 1;
    }
    else {
        dat = 0;
    }

    return dat;
}

/***********************************************************************
 * Name: vI2C_start_transmission()
 * Note: Start I2C Bus, SDA change to low when SCL is hight
 * Para: None
 * Return : None
************************************************************************/
static void vI2C_start_transmission(void)
{
    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_sda(1);
    vI2C_set_scl(1);
    vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
    vI2C_set_sda(0);
    //vI2C_ticks_delay(0);
    vI2C_set_scl(0);
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
    //vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
    vI2C_set_scl(0);
    vI2C_set_sda(0);
    vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
    vI2C_set_scl(1);
    vI2C_set_sda(1);
}

/***********************************************************************
 * Name: xI2C_write_byte(dat)
 * Note: Write 8bite data and get ack;
 * Para: dat -> the data which will be send out
 * Return : ack -> ack signal
************************************************************************/
static uint8_t xI2C_write_byte(uint8_t dat)
{
    uint8_t i;
    uint8_t ack;

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    for(i=0; i<8; i++)
    {
        if(dat & 0x80)
        {
            vI2C_set_sda(1);
        }
        else
        {
            vI2C_set_sda(0);
        }
        vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
        vI2C_set_scl(1);
        vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
        dat <<= 1;
        vI2C_set_scl(0);
    }

    vI2C_set_sda_dir(I2C_DIR_INPUT);
    vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
    vI2C_set_scl(1);
    vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
    ack = xI2C_read_sda_state();
    vI2C_set_scl(0);

    return ack;
}

/***********************************************************************
 * Name: xI2C_read_byte
 * Note: Read 8bite data and Send  ack;
 * Para: ack=0 -> Set ack, ack=1 -> Set noack
 * Return : read data
************************************************************************/
static uint8_t xI2C_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t dat;

    vI2C_set_sda_dir(I2C_DIR_INPUT);

    for(i=0; i<8; i++)
    {
        dat <<= 1;
        vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
        vI2C_set_scl(1);
        vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
        if(xI2C_read_sda_state())
        {
            dat |= 1;
        }
        vI2C_set_scl(0);
    }

    vI2C_set_sda_dir(I2C_DIR_OUTPUT);
    vI2C_set_sda(ack);    // ack = 0; ask, ack = 1,stop
    vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
    vI2C_set_scl(1);
    vI2C_ticks_delay(IIC_TIME_DELAY_TICKS);
    vI2C_set_scl(0);

    return dat;
}

#include <stdio.h>
/***********************************************************************
 * Name: xI2C_write_sequence
 * Note: Write data to device
 * Para: device -> device address, buf->subaddress + data
 * Return : how many bytes have been write
************************************************************************/
uint8_t xI2C_write_sequence(uint8_t device, uint8_t *buf, uint8_t cnt)
{
    uint8_t i = 0;

    if(xSemaphoreTakeRecursive(xRecMutexI2CSequence, portMAX_DELAY) == pdTRUE)
    {
        vI2C_start_transmission();
        if(xI2C_write_byte((device<<1)&(~0x01)) == 0)   //shift device addres left by 1 and clear last bit (write op)
        {
            for(i=0; i<cnt; i++)
            {
                if(xI2C_write_byte(*buf++))
                {
                    break;
                }
            }
        }
        else {
            // nack for given device address byte, end i2c communication
        }

        vI2C_stop_transmission();
        xSemaphoreGiveRecursive(xRecMutexI2CSequence);
    }

    return i;
}

/***********************************************************************
 * Name: xI2C_read_sequence
 * Note: Read data from device
 * Para: device->device address, subaddr->subaddress, acnt->subaddress lengh
 *       buf->read out data space, bcnt->read out data lengh
 * Return : write subaddress lengh
************************************************************************/
uint8_t xI2C_read_sequence(uint8_t device, uint8_t *subaddr, uint8_t acnt, uint8_t *buf, uint8_t bcnt)
{
    uint8_t i = 0;
    uint8_t wlen = 0;
    
    if(xSemaphoreTakeRecursive(xRecMutexI2CSequence, portMAX_DELAY) == pdTRUE)
    {
        vI2C_start_transmission();
        if(xI2C_write_byte((device<<1)&(~0x01)) == 0)   //shift device addres left by 1 and clear last bit (write op)
        {
            for (i=0; i<acnt; i++)
            {
                if(xI2C_write_byte(*subaddr++))
                {
                    break;
                }
            }

            wlen = i;
            if(i == acnt) {
                vI2C_start_transmission();
                xI2C_write_byte((device<<1)|0x01);   //shift device addres left by 1 and add read bit

                for(i=0; i<bcnt-1; i++)
                {
                    *buf++ = xI2C_read_byte(0); // read & send ACK
                }
                *buf = xI2C_read_byte(1); // read & send noack
            }
            else {
                // fail to write all given bytes, abort
            }
        }
        else {
            // nack for given device address byte, end i2c communication
        }

        vI2C_stop_transmission();
        xSemaphoreGiveRecursive(xRecMutexI2CSequence);
    }

    return wlen;
}

void vI2C_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //SCL signal
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //SDA signal
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}




/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
