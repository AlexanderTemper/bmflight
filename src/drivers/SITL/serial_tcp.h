#pragma once

#include <pthread.h>
#include "io/serial.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>

#define BASE_PORT 5701
typedef struct {
    int serverfd;
    int connfd;
//    dyad_Stream *conn;
    pthread_mutex_t txLock;
    pthread_mutex_t rxLock;
    bool connected;
} tcpPort_t;

typedef void (*writeCallbackFuncPtr)(void);
typedef void (*readCallbackFuncPtr)(uint8_t data);

void tcp_serial_initialize(serialPort_t *instance, tcpPort_t *stcpPort);
void tcp_initialize_server(tcpPort_t *s);
void tcp_initialize_client(tcpPort_t *s);

void tcp_serialWrite(serialPort_t *instance, uint8_t ch);
uint8_t tcp_serialRead(serialPort_t *instance);
uint32_t tcp_serialTotalRxWaiting(const serialPort_t *instance);
uint32_t tcp_serialTotalTxFree(const serialPort_t *instance);
bool tcp_isSerialTransmitBufferEmpty(const serialPort_t *instance);

void tcp_beginWrite(serialPort_t *instance);
void tcp_endWrite(serialPort_t *instance);
