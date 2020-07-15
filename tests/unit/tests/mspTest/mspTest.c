#include <stdio.h>
#include <unistd.h>

#include "sput.h"

#include "drivers/serial.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"

#include "common/fc.h"
#define BUFFER_SIZE 256
uint8_t rxBuffer[BUFFER_SIZE];
uint8_t txBuffer[BUFFER_SIZE];
/**
 * do not send any data
 * @param instance
 */
static void dummySerialWrite(serialPort_t *instance) {
}

/**
 * clear test buffer
 */
static void clearBuffer(void) {
    memset(rxBuffer, 0, BUFFER_SIZE);
    memset(txBuffer, 0, BUFFER_SIZE);
}

static void initSerial(serialPort_t *instance) {
    clearBuffer();
    instance->txBuffer = txBuffer;
    instance->txBufferHead = 0;
    instance->txBufferTail = 0;
    instance->txBufferSize = BUFFER_SIZE;
    instance->rxBuffer = rxBuffer;

    instance->rxBufferHead = 0;
    instance->rxBufferTail = 0;
    instance->rxBufferSize = BUFFER_SIZE;

    instance->serialWrite = dummySerialWrite;
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
 * generate a reqest on mspPort of type cmd
 * @param cmd
 * @param mspPort
 */
static void generateRequest(uint8_t cmd, mspPort_t *mspPort) {
    writeRxBuffer(mspPort->port, '$');
    writeRxBuffer(mspPort->port, 'M');
    writeRxBuffer(mspPort->port, '<');
    writeRxBuffer(mspPort->port, 0);
    writeRxBuffer(mspPort->port, cmd);
    writeRxBuffer(mspPort->port, cmd);
}

/**
 * simple command test
 */
static void test_simple(void) {
    serialPort_t testSerial;
    initSerial(&testSerial);
    mspPort_t mspPort;
    mspInit(&mspPort, &testSerial);

    //send MSP_API_VERSION over MSP V1
    writeRxBuffer(&testSerial, '$');
    mspProcess(&mspPort);
    sput_fail_unless(mspPort.c_state == MSP_HEADER_START, "MSP_HEADER_START");
    writeRxBuffer(&testSerial, 'M');
    mspProcess(&mspPort);
    sput_fail_unless(mspPort.c_state == MSP_HEADER_M, "MSP_HEADER_M");
    //error so back to idle
    writeRxBuffer(&testSerial, '!');
    mspProcess(&mspPort);
    sput_fail_unless(mspPort.c_state == MSP_IDLE, "MSP_IDLE after error");

    //start
    writeRxBuffer(&testSerial, '$');
    mspProcess(&mspPort);
    sput_fail_unless(mspPort.c_state == MSP_HEADER_START, "MSP_HEADER_START");
    writeRxBuffer(&testSerial, 'M');
    mspProcess(&mspPort);
    sput_fail_unless(mspPort.c_state == MSP_HEADER_M, "MSP_HEADER_M");
    writeRxBuffer(&testSerial, '<');
    mspProcess(&mspPort);
    uint8_t crc = 0;
    writeRxBuffer(&testSerial, 0);
    crc ^= 1;
    writeRxBuffer(&testSerial, 1);
    writeRxBuffer(&testSerial, crc);
    mspProcess(&mspPort);
    sput_fail_unless(mspPort.c_state == MSP_IDLE, "MSP_IDLE");
    sput_fail_unless(mspPort.cmdMSP == 1, "MSP_API_VERSION decode");
}

/**
 * test pushing a command
 */
static void test_push_msp(void) {
    serialPort_t testSerial;
    initSerial(&testSerial);

    mspPort_t mspPort;
    mspInit(&mspPort, &testSerial);

    mspSerialPush(&mspPort, MSP_API_VERSION, 0, 0, MSP_DIRECTION_REQUEST);

    bool ok = true;
    ok &= (txBuffer[0] == '$');
    ok &= (txBuffer[1] == 'M');
    ok &= (txBuffer[2] == '>');
    ok &= (txBuffer[3] == 0);
    ok &= (txBuffer[4] == MSP_API_VERSION);
    ok &= (txBuffer[5] == 1);

    sput_fail_unless(ok, "MSP_API_VERSION Request OK");
    testSerial.txBufferTail = testSerial.txBufferHead = 0; //Reset Buffer
    clearBuffer();

    uint8_t data[3] = { 'A', 'B', 'C' };
    uint8_t crc = 0;
    crc ^= 3;
    crc ^= MSP_API_VERSION;
    crc ^= data[0];
    crc ^= data[1];
    crc ^= data[2];

    sput_fail_unless(ok, "MSP_API_VERSION REPLY OK");
}

/**
 * read byte from tx Buffer
 * @param instance
 * @return
 */
static uint8_t readTxBuffer(serialPort_t *instance) {
    uint8_t ch = instance->txBuffer[instance->txBufferTail];
    if (instance->txBufferTail + 1 >= instance->txBufferSize) {
        instance->txBufferTail = 0;
    } else {
        instance->txBufferTail++;
    }
    //printf("Read: %c [0x%x]\n", ch, ch);
    return ch;
}

/**
 * calculate CRC
 * @param data
 * @param len
 * @return
 */
static uint8_t getCRC(const uint8_t *data, int len) {
    uint8_t checksum = 0;
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}

/**
 * assert function if two byte are same
 * @param a
 * @param b
 * @return
 */
static bool isSame(uint8_t a, uint8_t b) {
    if (a == b) {
        return true;
    }

    printf("Not Same: %c [0x%x] != %c [0x%x] \n", a, a, b, b);
    return false;
}

/**
 * number of bytes in tx buffer
 * @param instance
 * @return
 */
static uint32_t serialTXBytesWaiting(const serialPort_t *instance) {
    if (instance->txBufferHead >= instance->txBufferTail) {
        return instance->txBufferHead - instance->txBufferTail;
    } else {
        return instance->txBufferSize + instance->txBufferHead - instance->txBufferTail;
    }
}

/**
 * check data if an command is ok
 * @param instance
 * @param data
 * @param len
 * @return
 */
static bool checkReply(serialPort_t *instance, uint8_t *data, uint32_t len) {
    uint8_t *dataBase = data;
    if (serialTXBytesWaiting(instance) < (len + 4)) {
        sput_fail_unless(false, "tx buffer lenght");
        return false;
    }
    bool ok = true;
    ok &= isSame(readTxBuffer(instance), '$');
    ok &= isSame(readTxBuffer(instance), 'M');
    ok &= isSame(readTxBuffer(instance), '>');
    int i = len;
    while (i-- > 0) {
        if (!isSame(readTxBuffer(instance), *data++)) {
            return false;
        }
    }

    bool crcok = isSame(readTxBuffer(instance), getCRC(dataBase, len));
    if (!crcok) {
        sput_fail_unless(false, "crc ok");
        return false;
    }
    return ok;
}

/**
 * check if the reply of an command is ok
 * @param cmd
 * @param mspPort
 */
static void testReplyOfCommand(uint8_t cmd, mspPort_t *mspPort) {

    bool ok = false;
    switch (cmd) {
    case MSP_API_VERSION: {
        uint8_t data[5] = { 3, MSP_API_VERSION, MSP_PROTOCOL_VERSION, API_VERSION_MAJOR, API_VERSION_MINOR };
        ok = checkReply(mspPort->port, data, 5);
        break;
    }
    case MSP_FC_VARIANT: {
        uint8_t data[6] = { 4, MSP_FC_VARIANT, 'B', 'T', 'F', 'L' };
        ok = checkReply(mspPort->port, data, 6);
        break;
    }
    case MSP_FC_VERSION: {
        uint8_t data[5] = { 3, MSP_FC_VERSION, FC_VERSION_MAJOR, FC_VERSION_MINOR, FC_VERSION_PATCH_LEVEL };
        ok = checkReply(mspPort->port, data, 5);
        break;
    }
    default:
        sput_fail_unless(false, "command known");
        return;
    }
    sput_fail_unless(ok, "REPLY");
    sput_fail_unless(mspPort->c_state == MSP_IDLE, "MSP_IDLE");

}

static void test_reply(void) {
    serialPort_t testSerial;

    initSerial(&testSerial);
    mspPort_t mspPort;
    mspInit(&mspPort, &testSerial);
    generateRequest(MSP_API_VERSION, &mspPort);
    mspProcess(&mspPort);
    testReplyOfCommand(MSP_API_VERSION, &mspPort);

    sput_fail_unless(mspPort.cmdMSP == 1, "MSP_API_VERSION decode");
    sput_fail_unless(isSerialTransmitBufferEmpty(&testSerial), "tx buffer empty");

    generateRequest(MSP_API_VERSION, &mspPort);
    generateRequest(MSP_API_VERSION, &mspPort);
    mspProcess(&mspPort);
    testReplyOfCommand(MSP_API_VERSION, &mspPort);
    mspProcess(&mspPort);
    testReplyOfCommand(MSP_API_VERSION, &mspPort);
}

static serialPort_t commandSerialPort;
static mspPort_t commandMspPort;

static void test_MSP_API_VERSION(void) {
    generateRequest(MSP_API_VERSION, &commandMspPort);
    mspProcess(&commandMspPort);
    testReplyOfCommand(MSP_API_VERSION, &commandMspPort);
}

static void test_MSP_FC_VARIANT(void) {
    generateRequest(MSP_FC_VARIANT, &commandMspPort);
    mspProcess(&commandMspPort);
    testReplyOfCommand(MSP_FC_VARIANT, &commandMspPort);
}

static void test_MSP_FC_VERSION(void) {
    generateRequest(MSP_FC_VERSION, &commandMspPort);
    mspProcess(&commandMspPort);
    testReplyOfCommand(MSP_FC_VERSION, &commandMspPort);
}

int main(int argc, char *argv[]) {
    sput_start_testing()
    ;

    sput_enter_suite("Test MSP");
    sput_run_test(test_simple);

    sput_enter_suite("Test msp push");
    sput_run_test(test_push_msp);

    sput_enter_suite("Test reply");
    sput_run_test(test_reply);

    //test Commands with one Port
    initSerial(&commandSerialPort);
    mspInit(&commandMspPort, &commandSerialPort);

    sput_enter_suite("Test MSP_API_VERSION");
    sput_run_test(test_MSP_API_VERSION);

    sput_enter_suite("Test MSP_FC_VARIANT");
    sput_run_test(test_MSP_FC_VARIANT);

    sput_enter_suite("Test MSP_FC_VERSION");
    sput_run_test(test_MSP_FC_VERSION);


    sput_finish_testing()
    ;

    return sput_get_return_value();
}
