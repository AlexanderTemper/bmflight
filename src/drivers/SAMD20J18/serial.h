#pragma once

#include "io/serial.h"


typedef void (*writeCallbackFuncPtr)(void);
typedef void (*readCallbackFuncPtr)(uint8_t data);

void samd20j18_serial_initialize(writeCallbackFuncPtr wp,readCallbackFuncPtr rp);
bool samd20j18_serial_write(uint8_t *tx_data,uint16_t length);
