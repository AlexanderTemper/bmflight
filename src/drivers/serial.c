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

    if (!instance->blockWriteToHW) {
        // call serial to start transmission
        instance->serialWrite(instance);
    }
}

/**
 * send the data by calling serialWrite (blocks until all data is copied)
 * @param instance
 * @param data
 * @param count
 */
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count) {
    for (const uint8_t *p = data; count > 0; count--, p++) {

        while (!serialTxBytesFree(instance)) {
        };

        serialWrite(instance, *p);
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
    uint8_t ch;

    ch = instance->rxBuffer[instance->rxBufferTail];
    if (instance->rxBufferTail + 1 >= instance->rxBufferSize) {
        instance->rxBufferTail = 0;
    } else {
        instance->rxBufferTail++;
    }
    return ch;
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    return instance->txBufferHead == instance->txBufferTail;
}

/**
 * serialWrite will only fill the buffer until serialEndWrite gets called
 * @param instance
 */
void serialBeginWrite(serialPort_t *instance) {
    instance->blockWriteToHW = true;
}

/**
 * disable the block for serialWrite and trigger transmission
 * @param instance
 */
void serialEndWrite(serialPort_t *instance) {
    instance->blockWriteToHW = false;
    // trigger transmission
    instance->serialWrite(instance);
}

