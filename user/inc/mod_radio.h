#ifndef __MOD_RADIO_CONTROL_H__
#define __MOD_RADIO_CONTROL_H__

#include <stdbool.h>
#include "FreeRTOS.h"


extern void vRadioTask(void *pvArg);
extern void radio_recv_handler_irq(uint16_t);

#endif  // __MOD_RADIO_CONTROL_H__
