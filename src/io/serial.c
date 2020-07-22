#include "serial.h"

void serialPrint(serialPort_t *instance, const char *str) {
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

void serialWrite(serialPort_t *instance, uint8_t ch) {
    instance->serialWrite(instance, ch);
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
    return instance->serialTotalRxWaiting(instance);
}

/**
 * number if free bytes in tx buffer
 * Note we reserve one byte so the buffer can't get full
 * and the state Head==Tail always indicate an empty buffer
 * @param instance
 * @return
 */
uint32_t serialTxBytesFree(const serialPort_t *instance) {
    return instance->serialTotalTxFree(instance);
}

uint8_t serialRead(serialPort_t *instance) {
    return instance->serialRead(instance);
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    return instance->txBufferHead == instance->txBufferTail;
}

/**
 * serialWrite will only fill the buffer until serialEndWrite gets called
 * @param instance
 */
void serialBeginWrite(serialPort_t *instance) {
    instance->beginWrite(instance);
}

/**
 * disable the block for serialWrite and trigger transmission
 * @param instance
 */
void serialEndWrite(serialPort_t *instance) {
    instance->endWrite(instance);
}

