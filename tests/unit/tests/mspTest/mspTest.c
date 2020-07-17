#include <stdio.h>
#include <unistd.h>

#include "sput.h"
#include "fc.h"
#include "io/serial.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"

#include "common/debug.h"

#define BUFFER_SIZE 256
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

/**
 * do not send any data
 * @param instance
 */
static void dummySerialWrite(void) {
}

/**
 * @param cmd
 * @param reply
 * @return
 */
static mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply) {
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;
    switch (cmdMSP) {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;
    default:
        sput_fail_unless(false, "command known");
        break;
    }
    reply->result = MSP_RESULT_ACK;
    return ret;
}

static void initSerial(serialPort_t *instance) {
    instance->txBuffer = txBuffer;
    instance->txBufferHead = 0;
    instance->txBufferTail = 0;
    instance->txBufferSize = BUFFER_SIZE;
    instance->rxBuffer = rxBuffer;

    instance->rxBufferHead = 0;
    instance->rxBufferTail = 0;
    instance->rxBufferSize = BUFFER_SIZE;
    instance->triggerWrite = dummySerialWrite;
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
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
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
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspInit(&mspPort, &testSerial);

    mspSerialPush(&mspPort, MSP_API_VERSION, 0, 0, MSP_DIRECTION_REQUEST);

    bool ok = true;
    ok &= (readTxBuffer(&testSerial) == '$');
    ok &= (readTxBuffer(&testSerial) == 'M');
    ok &= (readTxBuffer(&testSerial) == '<');
    ok &= (readTxBuffer(&testSerial) == 0);
    ok &= (readTxBuffer(&testSerial) == MSP_API_VERSION);
    ok &= (readTxBuffer(&testSerial) == 1);

    sput_fail_unless(ok, "MSP_API_VERSION Request OK");
    testSerial.txBufferTail = testSerial.txBufferHead = 0; //Reset Buffer
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
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
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

//static serialPort_t commandSerialPort;
//static mspPort_t commandMspPort;

//static void test_MSP_API_VERSION(void) {
//    generateRequest(MSP_API_VERSION, &commandMspPort);
//    mspProcess(&commandMspPort);
//    testReplyOfCommand(MSP_API_VERSION, &commandMspPort);
//}
//
//static void test_MSP_FC_VARIANT(void) {
//    generateRequest(MSP_FC_VARIANT, &commandMspPort);
//    mspProcess(&commandMspPort);
//    testReplyOfCommand(MSP_FC_VARIANT, &commandMspPort);
//}
//
//static void test_MSP_FC_VERSION(void) {
//    generateRequest(MSP_FC_VERSION, &commandMspPort);
//    mspProcess(&commandMspPort);
//    testReplyOfCommand(MSP_FC_VERSION, &commandMspPort);
//}

static void test_printDebug(void) {
    serialPort_t testSerial;
    initSerial(&testSerial);
    mspPort_t mspPort;
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspInit(&mspPort, &testSerial);
    initMspDebugPort(&mspPort);
    initDebug(&mspDebugData);

    printDebug("test");
    static uint8_t data[6] = { 4, MSP_DEBUGMSG, 't', 'e', 's', 't' }; //TOD warum muss hier static sein ?????

    bool ok = checkReply(mspPort.port, data, 6);
    sput_fail_unless(ok, "REPLY");
    sput_fail_unless(mspPort.c_state == MSP_IDLE, "MSP_IDLE");

    static uint8_t testdata[3] = { 'A', 'B', 'C' };
    static uint8_t data2[5] = { 3, MSP_DEBUGMSG, 'A', 'B', 'C' };
    debugData(testdata, 3);

    ok = checkReply(mspPort.port, data2, 5);
    sput_fail_unless(ok, "REPLY");
    sput_fail_unless(mspPort.c_state == MSP_IDLE, "MSP_IDLE");
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

    sput_enter_suite("Test MSP_DEBUG");
    sput_run_test(test_printDebug);

    sput_finish_testing()
    ;

    return sput_get_return_value();
}
