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

void tcp_serial_initialize(writeCallbackFuncPtr wp,readCallbackFuncPtr rp);
bool tcp_serial_write(uint8_t *tx_data,uint16_t length);
