#include "drivers/SITL/serial_tcp.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>


static tcpPort_t tcpSerialPort;
static bool tcpPortInitialized;
static bool tcpStart = false;

#define BASE_PORT 5760


static readCallbackFuncPtr readCallback;

bool tcp_serial_write(uint8_t *tx_data,uint16_t length){
    //printf("try write \n");
    fwrite(tx_data,1,length,stdout);
    if (tcpSerialPort.conn == NULL){
        return false;
    }
    dyad_write(tcpSerialPort.conn, (const void *) &tx_data, length);
    //printf("write done \n");
    return true;
}

static void tcpDataIn(tcpPort_t *instance, uint8_t* ch, int size)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    while (size--) {
        readCallback(*(ch++));
    }
    pthread_mutex_unlock(&s->rxLock);
}
static void onData(dyad_Event *e) {
    tcpPort_t* s = (tcpPort_t*) (e->udata);
    tcpDataIn(s, (uint8_t*) e->data, e->size);
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


void tcp_serial_initialize(writeCallbackFuncPtr wp, readCallbackFuncPtr rp) {
    tcpReconfigure(&tcpSerialPort, 1);
    readCallback = rp;
}

