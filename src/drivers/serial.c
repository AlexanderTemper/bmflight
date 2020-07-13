#include <stdbool.h>
#include <stdint.h>


#include "serial.h"

void serialPrint(serialPort_t *instance, const char *str) {
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

void serialWrite(serialPort_t *instance, uint8_t ch) {
    instance->txBuffer[instance->txBufferHead] = ch;

    if (instance->txBufferHead + 1 >= instance->txBufferSize) {
        instance->txBufferHead = 0;
    } else {
        instance->txBufferHead++;
    }

    // call serial to start transmission
    instance->serialWrite(instance);
}

void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count) {
    if (instance->writeBuf) {
        instance->writeBuf(instance, data, count);
    } else {
        for (const uint8_t *p = data; count > 0; count--, p++) {

            while (!serialTxBytesFree(instance)) {
            };

            serialWrite(instance, *p);
        }
    }
}

uint32_t serialRxBytesWaiting(const serialPort_t *instance) {
    if (instance->rxBufferHead >= instance->rxBufferTail) {
        return instance->rxBufferHead - instance->rxBufferTail;
    } else {
        return instance->rxBufferSize + instance->rxBufferHead - instance->rxBufferTail;
    }
}

/**
 * number if free bytes in tx buffer
 * Note we reserve one byte so the buffer can't get full
 * and the state Head==Tail always indicate an empty buffer
 * @param instance
 * @return
 */
uint32_t serialTxBytesFree(const serialPort_t *instance) {
    uint32_t bytesUsed;

    if (instance->txBufferHead >= instance->txBufferTail) {
        bytesUsed = instance->txBufferHead - instance->txBufferTail;
    } else {
        bytesUsed = instance->txBufferSize + instance->txBufferHead - instance->txBufferTail;
    }

    // -1 so the buffer can never get full
    return (instance->txBufferSize - 1) - bytesUsed;
}

uint8_t serialRead(serialPort_t *instance) {
    return instance->serialRead(instance);
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    return instance->isSerialTransmitBufferEmpty(instance);
}

