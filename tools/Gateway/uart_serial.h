#pragma once

#include "io/serial.h"

bool uart_serial_initialize(serialPort_t *instance, const char *devName);

void uart_serialWrite(serialPort_t *instance, uint8_t ch);
uint8_t uart_serialRead(serialPort_t *instance);
uint32_t uart_serialTotalRxWaiting(const serialPort_t *instance);
uint32_t uart_serialTotalTxFree(const serialPort_t *instance);
bool uart_isSerialTransmitBufferEmpty(const serialPort_t *instance);

void uart_beginWrite(serialPort_t *instance);
void uart_endWrite(serialPort_t *instance);

void update_read(serialPort_t *instance);
