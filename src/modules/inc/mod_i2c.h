/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_I2C_H
#define MOD_I2C_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
    const uint8_t devAddr;
    const uint8_t ctrlReg;
    uint8_t* data;
    uint16_t dataSize;
} I2cParams_t;

extern xSemaphoreHandle xSemaphI2CLcdInitDone;

//extern uint8_t xI2C_write_sequence(uint8_t device, uint8_t *buf, uint8_t cnt);
//uint8_t xI2C_write_sequence(I2cParams_t* params);
extern uint8_t xI2C_write_sequence1(I2cParams_t* params);
extern uint8_t xI2C_read_sequence(I2cParams_t* params);

extern void vI2C_configuration(void);


#endif
