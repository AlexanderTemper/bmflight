#include "drivers/SITL/serial_tcp.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

static tcpPort_t tcpSerialPort;
static serialPort_t *serialPort;
static bool tcpPortInitialized;
static bool tcpStart = false;

#define BASE_PORT 5760

void tcp_serialWrite(serialPort_t *instance, uint8_t ch) {
    pthread_mutex_lock(&tcpSerialPort.txLock);

    instance->txBuffer[instance->txBufferHead] = ch;
    if (instance->txBufferHead + 1 >= instance->txBufferSize) {
        instance->txBufferHead = 0;
    } else {
        instance->txBufferHead++;
    }
    pthread_mutex_unlock(&tcpSerialPort.txLock);

    instance->txBufferTail = instance->txBufferHead;

    if (tcpSerialPort.conn == NULL) {
        fwrite(&ch, 1, 1, stdout);
        return;
    }

    pthread_mutex_lock(&tcpSerialPort.txLock);

    if (instance->txBufferHead < instance->txBufferTail) {
        // send data till end of buffer
        int chunk = instance->txBufferSize - instance->txBufferTail;
        dyad_write(tcpSerialPort.conn, (const void *) &instance->txBuffer[instance->txBufferTail], chunk);
        instance->txBufferTail = 0;
    }
    int chunk = instance->txBufferHead - instance->txBufferTail;
    if (chunk) {
        dyad_write(tcpSerialPort.conn, (const void*) &instance->txBuffer[instance->txBufferTail], chunk);
    }
    instance->txBufferTail = instance->txBufferHead;

    pthread_mutex_unlock(&tcpSerialPort.txLock);
}
uint8_t tcp_serialRead(serialPort_t *instance) {
    uint8_t ch;

    pthread_mutex_lock(&tcpSerialPort.rxLock);

    ch = instance->rxBuffer[instance->rxBufferTail];
    if (instance->rxBufferTail + 1 >= instance->rxBufferSize) {
        instance->rxBufferTail = 0;
    } else {
        instance->rxBufferTail++;
    }
    pthread_mutex_unlock(&tcpSerialPort.rxLock);

    return ch;
}
uint32_t tcp_serialTotalRxWaiting(const serialPort_t *instance) {
    int32_t count;
    pthread_mutex_lock(&tcpSerialPort.rxLock);
    if (instance->rxBufferHead >= instance->rxBufferTail) {
        count = instance->rxBufferHead - instance->rxBufferTail;
    } else {
        count = instance->rxBufferSize + instance->rxBufferHead - instance->rxBufferTail;
    }
    pthread_mutex_unlock(&tcpSerialPort.rxLock);

    return count;
}
uint32_t tcp_serialTotalTxFree(const serialPort_t *instance) {
    uint32_t bytesUsed;
    pthread_mutex_lock(&tcpSerialPort.txLock);
    if (instance->txBufferHead >= instance->txBufferTail) {
        bytesUsed = instance->txBufferHead - instance->txBufferTail;
    } else {
        bytesUsed = instance->txBufferSize + instance->txBufferHead - instance->txBufferTail;
    }
    uint32_t bytesFree = (instance->txBufferSize - 1) - bytesUsed;
    pthread_mutex_unlock(&tcpSerialPort.txLock);
    return bytesFree;
}
bool tcp_isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    pthread_mutex_lock(&tcpSerialPort.txLock);
    bool isEmpty = instance->txBufferTail == instance->txBufferHead;
    pthread_mutex_unlock(&tcpSerialPort.txLock);
    return isEmpty;
}

void tcp_beginWrite(serialPort_t *instance) {

}
void tcp_endWrite(serialPort_t *instance) {

}

static void tcpDataIn(serialPort_t *instance, uint8_t* ch, int size) {

    pthread_mutex_lock(&tcpSerialPort.rxLock);
    while (size--) {
        //        printf("%c", *ch);
        instance->rxBuffer[instance->rxBufferHead] = *(ch++);
        if (instance->rxBufferHead + 1 >= instance->rxBufferSize) {
            instance->rxBufferHead = 0;
        } else {
            instance->rxBufferHead++;
        }
    }
    pthread_mutex_unlock(&tcpSerialPort.rxLock);
}

static void onData(dyad_Event *e) {
    tcpDataIn(serialPort, (uint8_t*) e->data, e->size);
}

static void onClose(dyad_Event *e) {
    tcpPort_t* s = (tcpPort_t*) (e->udata);
    s->clientCount--;
    s->conn = NULL;
    fprintf(stderr, "[CLS]UART%u: %d,%d\n", s->id + 1, s->connected, s->clientCount);
    if (s->clientCount == 0) {
        s->connected = false;
    }
}
static void onAccept(dyad_Event *e) {
    tcpPort_t* s = (tcpPort_t*) (e->udata);
    fprintf(stderr, "New connection on UART%u, %d\n", s->id + 1, s->clientCount);

    s->connected = true;
    if (s->clientCount > 0) {
        dyad_close(e->remote);
        return;
    }
    s->clientCount++;
    fprintf(stderr, "[NEW]UART%u: %d,%d\n", s->id + 1, s->connected, s->clientCount);
    s->conn = e->remote;
    dyad_setNoDelay(e->remote, 1);
    dyad_setTimeout(e->remote, 120);
    dyad_addListener(e->remote, DYAD_EVENT_DATA, onData, e->udata);
    dyad_addListener(e->remote, DYAD_EVENT_CLOSE, onClose, e->udata);
}
static void tcpReconfigure(tcpPort_t *s, int id) {
    if (tcpPortInitialized) {
        fprintf(stderr, "port is already initialized!\n");
        return;
    }

    if (pthread_mutex_init(&s->txLock, NULL) != 0) {
        fprintf(stderr, "TX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return;
    }
    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        fprintf(stderr, "RX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return;
    }

    tcpStart = true;
    tcpPortInitialized = true;

    s->connected = false;
    s->clientCount = 0;
    s->id = id;
    s->conn = NULL;
    s->serv = dyad_newStream();
    dyad_setNoDelay(s->serv, 1);
    dyad_addListener(s->serv, DYAD_EVENT_ACCEPT, onAccept, s);

    if (dyad_listenEx(s->serv, NULL, BASE_PORT, 10) == 0) {
        fprintf(stderr, "bind port %u for UART\n", (unsigned) BASE_PORT);
    } else {
        fprintf(stderr, "bind port %u for UART failed!!\n", (unsigned) BASE_PORT);
    }
}

void tcp_serial_initialize(serialPort_t *instance) {
    serialPort = instance;
    tcpReconfigure(&tcpSerialPort, 1);
}

