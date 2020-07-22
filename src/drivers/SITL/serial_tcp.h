#pragma once

#include <pthread.h>
#include "io/serial.h"

#include "dyad.h"
typedef struct {
    dyad_Stream *serv;
    dyad_Stream *conn;
    pthread_mutex_t txLock;
    pthread_mutex_t rxLock;
    bool connected;
    uint16_t clientCount;
    uint8_t id;
} tcpPort_t;

typedef void (*writeCallbackFuncPtr)(void);
typedef void (*readCallbackFuncPtr)(uint8_t data);


void tcp_serial_initialize(serialPort_t *instance);

void tcp_serialWrite(serialPort_t *instance, uint8_t ch);
uint8_t tcp_serialRead(serialPort_t *instance);
uint32_t tcp_serialTotalRxWaiting(const serialPort_t *instance);
uint32_t tcp_serialTotalTxFree(const serialPort_t *instance);
bool tcp_isSerialTransmitBufferEmpty(const serialPort_t *instance);

void tcp_beginWrite(serialPort_t *instance);
void tcp_endWrite(serialPort_t *instance);
