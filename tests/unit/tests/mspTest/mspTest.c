#include <stdio.h>
#include <unistd.h>

#include "sput.h"

#include "drivers/serial.h"
#include "msp/msp.h"
#include "common/utils.h"
#define BUFFER_SIZE 256

/**
 * do not send any data
 * @param instance
 */
static void dummySerialWrite(serialPort_t *instance) {
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
 * print string in rx buffer
 * @param instance
 * @param str
 */
static void printRxBuffer(serialPort_t *instance, const char *str) {
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        writeRxBuffer(instance, ch);
    }
}
uint8_t rxBuffer[BUFFER_SIZE];
uint8_t txBuffer[BUFFER_SIZE];

/**
 * test wrap an filling of tx buffer
 */
static void testMsp(void) {
    serialPort_t testSerial;
    testSerial.txBuffer = txBuffer;
    testSerial.txBufferHead = 0;
    testSerial.txBufferTail = 0;
    testSerial.txBufferSize = BUFFER_SIZE;
    testSerial.rxBuffer = rxBuffer;

    testSerial.rxBufferHead = 0;
    testSerial.rxBufferTail = 0;
    testSerial.rxBufferSize = BUFFER_SIZE;

    testSerial.serialWrite = dummySerialWrite;

    mspPort_t mspPort;
    mspInit(&mspPort, &testSerial);

    //send MSP_API_VERSION over MSP V1
    printRxBuffer(&testSerial, "$M<");
    uint8_t crc = 0;
    writeRxBuffer(&testSerial, 0);
    crc ^= 1;
    writeRxBuffer(&testSerial, 1);
    writeRxBuffer(&testSerial, crc);
    mspProcess(&mspPort);

    sput_fail_unless(mspPort.c_state == MSP_COMMAND_RECEIVED, "MSP_COMMAND_RECEIVED");
    sput_fail_unless(mspPort.cmdMSP == 1, "MSP_API_VERSION decode");

}

int main(int argc, char *argv[]) {
    sput_start_testing()
    ;

    sput_enter_suite("Test MSP");
    sput_run_test(testMsp);
    sput_finish_testing()
    ;

    return sput_get_return_value();
}
