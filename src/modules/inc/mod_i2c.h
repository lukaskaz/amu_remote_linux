/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_I2C_H
#define MOD_I2C_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

extern xSemaphoreHandle xRecMutexI2CSequence;
extern xSemaphoreHandle xSemaphI2CLcdInitDone;

extern uint8_t xI2C_write_sequence(uint8_t device, uint8_t *buf, uint8_t cnt);
extern uint8_t xI2C_read_sequence(uint8_t device, uint8_t *subaddr, uint8_t acnt, uint8_t *buf, uint8_t bcnt);

extern void vI2C_configuration(void);


#endif
