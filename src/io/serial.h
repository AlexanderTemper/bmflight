#pragma once

#include "global.h"

typedef struct serialPort_s serialPort_t;

struct serialPort_s {
    // ringbuffer internals
    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    uint8_t *rxBuffer;
    uint8_t *txBuffer;
    // indicates the next free field in the buffer
    volatile uint32_t rxBufferHead;
    volatile uint32_t txBufferHead;
    // indicates the start of the data in the buffer
    volatile uint32_t rxBufferTail;
    volatile uint32_t txBufferTail;

    // if true only the buffer gets filled by serialWrite can be enabled and disabled by serialBeginWrite and serialEndWrite
    void (*serialWrite)(serialPort_t *instance, uint8_t ch);
    uint8_t (*serialRead)(serialPort_t *instance);
    uint32_t (*serialTotalRxWaiting)(const serialPort_t *instance);
    uint32_t (*serialTotalTxFree)(const serialPort_t *instance);
    bool (*isSerialTransmitBufferEmpty)(const serialPort_t *instance);
    // Optional functions used to buffer large writes.
    void (*beginWrite)(serialPort_t *instance);
    void (*endWrite)(serialPort_t *instance);
};

void serialWrite(serialPort_t *instance, uint8_t ch);
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count);
uint8_t serialRead(serialPort_t *instance);
uint32_t serialTxBytesFree(const serialPort_t *instance);
uint32_t serialRxBytesWaiting(const serialPort_t *instance);
bool isSerialTransmitBufferEmpty(const serialPort_t *instance);
void serialPrint(serialPort_t *instance, const char *str);
void serialBeginWrite(serialPort_t *instance);
void serialEndWrite(serialPort_t *instance);
