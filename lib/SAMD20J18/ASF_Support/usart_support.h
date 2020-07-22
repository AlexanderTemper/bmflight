#pragma once

#include "io/serial.h"

void samd20j18_serial_initialize(serialPort_t *instance);

void samd20j18_serialWrite(serialPort_t *instance, uint8_t ch);
uint8_t samd20j18_serialRead(serialPort_t *instance);
uint32_t samd20j18_serialTotalRxWaiting(const serialPort_t *instance);
uint32_t samd20j18_serialTotalTxFree(const serialPort_t *instance);
bool samd20j18_isSerialTransmitBufferEmpty(const serialPort_t *instance);

void samd20j18_beginWrite(serialPort_t *instance);
void samd20j18_endWrite(serialPort_t *instance);
