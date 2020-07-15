#pragma once

#include "platform.h"

typedef struct serialPort_s serialPort_t;

struct serialPort_s {

    // ringbuffer internals
    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    // indicates the next free field in the buffer
    volatile uint32_t rxBufferHead;
    volatile uint32_t txBufferHead;
    // indicates the start of the data in the buffer
    volatile uint32_t rxBufferTail;
    volatile uint32_t txBufferTail;

    // if true only the buffer gets filled by serialWrite can be enabled and disabled by serialBeginWrite and serialEndWrite
    bool blockWriteToHW;
    // api calls
    void (*serialWrite)(serialPort_t *instance);
};


typedef struct serialDevice_s {
    serialPort_t port;
    volatile uint8_t rxBuffer[64];
    volatile uint8_t txBuffer[64];
} serialDevice_t;

void serialWrite(serialPort_t *instance, uint8_t ch);
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count);
uint8_t serialRead(serialPort_t *instance);
uint32_t serialTxBytesFree(const serialPort_t *instance);
uint32_t serialRxBytesWaiting(const serialPort_t *instance);
bool isSerialTransmitBufferEmpty(const serialPort_t *instance);
void serialPrint(serialPort_t *instance, const char *str);
void serialBeginWrite(serialPort_t *instance);
void serialEndWrite(serialPort_t *instance);
