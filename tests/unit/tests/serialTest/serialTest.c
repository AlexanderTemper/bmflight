#include <stdio.h>
#include <unistd.h>

#include "sput.h"

#include "drivers/serial.h"
#include "common/utils.h"
#include "stopWatch.h"

#define BUFFER_SIZE 256

/**
 * test if two arrays are equal
 * @param first
 * @param second
 * @return
 */
static bool testArray(uint8_t *first, uint8_t *second) {

    for (int e = 0; e < BUFFER_SIZE; e++) {
        if (first[e] != second[e]) {
            //printf("[%d,%d]%d\n",first[e],first[e],e);
            return false;
        }
    }
    return true;
}

/**
 * do not send any data
 * @param instance
 */
static void dummySerialWrite(serialPort_t *instance) {
}

/**
 * send data after ~ 200us
 * @param instance
 */
static void emulateTransmit(serialPort_t *instance) {
    static stopWatch_t watch;
    // nothing to transmit
    if (instance->txBufferHead == instance->txBufferTail) {
        return;
    }

    if (!watch.running) {
        startTimer(&watch); // start timer
    }

    if (getTimeUsec(&watch) > 200) { //200us
        if (instance->txBufferHead > instance->txBufferTail) {
            instance->txBufferTail = instance->txBufferHead;
        } else {
            instance->txBufferTail = 0;
        }
        startTimer(&watch); // start timer
    }
}

uint8_t rxBuffer[BUFFER_SIZE];
uint8_t txBuffer[BUFFER_SIZE];
uint8_t txRef[BUFFER_SIZE];
uint8_t rxRef[BUFFER_SIZE];

/**
 * test wrap an filling of tx buffer
 */
static void test_buffer_wrap(void) {
    serialPort_t testSerial;
    testSerial.txBuffer = txBuffer;
    testSerial.txBufferHead = 0;
    testSerial.txBufferTail = 0;
    testSerial.txBufferSize = BUFFER_SIZE;

    testSerial.serialWrite = dummySerialWrite;

    //fill whole Buffer
    uint8_t d = 1;
    int i = 0;
    while (serialTxBytesFree(&testSerial) > 0) {
        txRef[i++] = d;
        serialWrite(&testSerial, d++);
    }
    sput_fail_unless(serialTxBytesFree(&testSerial) == 0, "buffer filled");
    sput_fail_unless(testArray(txBuffer, txRef), "Buffer Test");
    sput_fail_unless(testSerial.txBufferHead == BUFFER_SIZE-1, "txBufferHead");
    sput_fail_unless(testSerial.txBufferTail == 0, "txBufferTail");

    // emulate first transmission of first half of buffer
    testSerial.txBufferTail = BUFFER_SIZE / 2;

    sput_fail_unless(serialTxBytesFree(&testSerial) == BUFFER_SIZE/2, "emulate transmission");
    sput_fail_unless(testSerial.txBufferHead == BUFFER_SIZE-1, "txBufferHead");
    sput_fail_unless(testSerial.txBufferTail == BUFFER_SIZE / 2, "txBufferTail");

    // [1,2,3,4....T.....H] => [a2,3,4....T.....a]
    // write new data
    // fill last byte
    txRef[BUFFER_SIZE - 1] = 'a';
    serialWrite(&testSerial, 'a');

    i = 0;
    while (serialTxBytesFree(&testSerial) > 0) { //fill tx buffer
        txRef[i++] = 'a';
        serialWrite(&testSerial, 'a');
    }

    sput_fail_unless(serialTxBytesFree(&testSerial) == 0, "write data");
    sput_fail_unless(testArray(txBuffer, txRef), "Buffer Test");
    sput_fail_unless(testSerial.txBufferHead == BUFFER_SIZE / 2-1, "txBufferHead");
    sput_fail_unless(testSerial.txBufferTail == BUFFER_SIZE / 2, "txBufferTail");
}

/**
 * test serialWrite with fake write callback
 */
static void test_running(void) {
    serialPort_t testSerial;
    testSerial.txBuffer = txBuffer;
    testSerial.txBufferHead = 0;
    testSerial.txBufferTail = 0;
    testSerial.txBufferSize = BUFFER_SIZE;

    //Data is send after 200uS
    testSerial.serialWrite = emulateTransmit;

    // send Some Data
    serialWrite(&testSerial, 'a');
    serialWrite(&testSerial, 'a');

    sput_fail_unless(testSerial.txBufferHead == 2, "txBufferHead");
    sput_fail_unless(testSerial.txBufferTail == 0, "txBufferTail");
    sput_fail_unless(serialTxBytesFree(&testSerial) == BUFFER_SIZE-3, "data before wait");
    while (serialTxBytesFree(&testSerial) < BUFFER_SIZE - 1) { // wait for transmit fake
        emulateTransmit(&testSerial); //gets called by HW
    }
    sput_fail_unless(testSerial.txBufferHead == 2, "txBufferHead");
    sput_fail_unless(testSerial.txBufferTail == 2, "txBufferTail");
}

/**
 * write a byte to the rx Buffer
 * @param instance
 * @param ch
 */
static void writeRxBuffer(serialPort_t *instance, uint8_t ch) {
    instance->rxBuffer[instance->rxBufferHead] = ch;
    if (instance->rxBufferHead + 1 >= instance->rxBufferSize) {
        instance->rxBufferHead = 0;
    } else {
        instance->rxBufferHead++;
    }
}

/**
 * test read of rx buffer
 */
static void test_read(void) {

    serialPort_t testSerial;
    testSerial.rxBuffer = rxBuffer;
    testSerial.rxBufferHead = 0;
    testSerial.rxBufferTail = 0;
    testSerial.rxBufferSize = BUFFER_SIZE;

    //fill whole Buffer
    for (int i = 0; i < BUFFER_SIZE - 1; i++) {
        rxRef[i] = i + 1;
        writeRxBuffer(&testSerial, i + 1);
    }
    sput_fail_unless(testArray(rxBuffer, rxRef), "Buffer Test");
    sput_fail_unless(testSerial.rxBufferHead == BUFFER_SIZE-1, "rxBufferHead");
    sput_fail_unless(testSerial.rxBufferTail == 0, "rxBufferTail");

    // read buffer
    bool readOK = true;
    for (int i = 0; serialRxBytesWaiting(&testSerial) > 0; i++) {
        uint8_t ch = serialRead(&testSerial);
        if (ch != (uint8_t) (i + 1)) {
            //printf("%d,%d\n",ch,i);
            readOK = false;
        }
    }
    sput_fail_unless(readOK, "readTest");
    sput_fail_unless(testSerial.rxBufferHead == BUFFER_SIZE-1, "rxBufferHead");
    sput_fail_unless(testSerial.rxBufferTail == BUFFER_SIZE-1, "rxBufferTail");
    sput_fail_unless(serialRxBytesWaiting(&testSerial) == 0, "bytes waiting");

    writeRxBuffer(&testSerial, 'a');
    writeRxBuffer(&testSerial, 'b');
    sput_fail_unless(testSerial.rxBufferHead == 1, "rxBufferHead");
    sput_fail_unless(testSerial.rxBufferTail == BUFFER_SIZE-1, "rxBufferTail");
    sput_fail_unless(serialRxBytesWaiting(&testSerial) == 2, "bytes waiting");

    //reset Buffer
    sput_fail_unless(serialRead(&testSerial) == 'a', "read ");
    sput_fail_unless(serialRead(&testSerial) == 'b', "read b");
    sput_fail_unless(serialRxBytesWaiting(&testSerial) == 0, "bytes waiting");

    // read wrap
    testSerial.rxBufferHead = BUFFER_SIZE / 2;
    testSerial.rxBufferTail = BUFFER_SIZE / 2;

    memset(rxBuffer, 0, BUFFER_SIZE);
    memset(rxRef, 0, BUFFER_SIZE);
    //fill whole Buffer
    for (int i = 0; i < BUFFER_SIZE - 1; i++) {
        rxRef[i] = i + 1;
        writeRxBuffer(&testSerial, i + 1);
    }
    sput_fail_unless(testSerial.rxBufferHead == BUFFER_SIZE/2 -1, "rxBufferHead");
    sput_fail_unless(testSerial.rxBufferTail == BUFFER_SIZE/2, "rxBufferTail");

    // read buffer
    readOK = true;
    for (int i = 0; serialRxBytesWaiting(&testSerial) > 0; i++) {
        uint8_t ch = serialRead(&testSerial);
        if (ch != (uint8_t) (i + 1)) {
            //printf("%d,%d\n",ch,i);
            readOK = false;
        }
    }
    sput_fail_unless(readOK, "readTest");
    sput_fail_unless(testSerial.rxBufferHead == BUFFER_SIZE/2 -1, "rxBufferHead");
    sput_fail_unless(testSerial.rxBufferTail == BUFFER_SIZE/2 -1, "rxBufferTail");
    sput_fail_unless(serialRxBytesWaiting(&testSerial) == 0, "bytes waiting");

}
int main(int argc, char *argv[]) {
    sput_start_testing()
    ;

    sput_enter_suite("No Wrap");
    sput_run_test(test_running);
    sput_enter_suite("Buffer Wrap");
    sput_run_test(test_buffer_wrap);
    sput_enter_suite("Buffer Read");
    sput_run_test(test_read);

    sput_finish_testing()
    ;

    return sput_get_return_value();
}
