#ifndef __MOD_I2C_H__
#define __MOD_I2C_H__

#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
    const uint8_t devAddr;
    const uint8_t ctrlReg;
    uint8_t* data;
    uint16_t dataSize;
} I2cParams_t;

extern uint8_t xI2C_write_sequence(I2cParams_t* params);
extern uint8_t xI2C_read_sequence(I2cParams_t* params);
extern uint8_t xI2C_read_sequence_slow(I2cParams_t* params);

extern void vI2C_configuration(void);


#endif  // __MOD_I2C_H__
