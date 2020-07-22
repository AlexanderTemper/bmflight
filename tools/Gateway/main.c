#include <stdio.h>   /* Standard input/output definitions */
#include "io/serial.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "common/debug.h"
#include "uart_serial.h"

static serialPort_t serialInstance;
static mspPort_t mspPort;

#define BUFFER_SIZE 256
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

static void debugStdout(const uint8_t* data, uint16_t len) {
    fwrite(data, 1, len, stdout);
}

static bool initialize_uart_serial(const char *devName) {
    serialInstance.txBuffer = txBuffer;
    serialInstance.txBufferHead = 0;
    serialInstance.txBufferTail = 0;
    serialInstance.txBufferSize = BUFFER_SIZE;
    serialInstance.rxBuffer = rxBuffer;

    serialInstance.rxBufferHead = 0;
    serialInstance.rxBufferTail = 0;
    serialInstance.rxBufferSize = BUFFER_SIZE;
    serialInstance.serialWrite = &uart_serialWrite;
    serialInstance.serialRead = &uart_serialRead;
    serialInstance.serialTotalRxWaiting = &uart_serialTotalRxWaiting;
    serialInstance.serialTotalTxFree = &uart_serialTotalTxFree;
    serialInstance.isSerialTransmitBufferEmpty = &uart_isSerialTransmitBufferEmpty;
    serialInstance.beginWrite = &uart_beginWrite;
    serialInstance.endWrite = &uart_endWrite;
    return uart_serial_initialize(&serialInstance, devName);

}
/**
 * @param cmd
 * @param reply
 * @return
 */
static mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply) {
    printf("mspFcProcessCommand\n");
    return MSP_RESULT_CMD_UNKNOWN;
}

/**
 * @param cmd
 * @param reply
 * @return
 */
static void mspFcProcessReply(mspPacket_t *cmd) {
    sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default

    switch (cmdMSP) {
    case MSP_API_VERSION: {
        uint8_t version = sbufReadU8(src);
        uint8_t version_major = sbufReadU8(src);
        uint8_t version_minor = sbufReadU8(src);

        printf("get Version Data %d %d %d\n", version, version_major, version_minor);
        break;
    }
    case MSP_RAW_IMU: {
        int16_t acc[3];
        int16_t gyro[3];
        for (int i = 0; i < 3; i++) {
            acc[i] = sbufReadU16(src);
        }

        for (int i = 0; i < 3; i++) {
            gyro[i] = sbufReadU16(src);
        }
        printf("get AccData [%d %d %d], GyroData [%d %d %d] \r", acc[X], acc[Y], acc[Z], gyro[X], gyro[Y], gyro[Z]);
        break;
    }
    case MSP_DEBUGMSG: {
        fwrite(src->ptr, 1, sbufBytesRemaining(src), stdout);
        //printf("heisl\n");
        break;
    }
    default:
        printf("WATT??\n");
        break;
    }
}

int main(int argc, char *argv[]) {

    initDebug(&debugStdout);

    if (argc == 3 && !strcmp(argv[1], "uart")) {
        printf("Start Uart Serial\n");
        if (!initialize_uart_serial(argv[2])) {
            return -1;
        }

    } else if (argc == 2 && !strcmp(argv[1], "tcp")) {
        printf("Start TCP Serial\n*** todo implement***\n");
        return -1;
    } else {
        printf("-------- Usage --------\n    gateway [uart] [devName]\n    or\n    gateway [tcp]\n");
        return -1;
    }

    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspPort.mspProcessReplyFnPtr = &mspFcProcessReply;
    mspInit(&mspPort, &serialInstance);

    while (1) {
        mspSerialPush(&mspPort, MSP_RAW_IMU, 0, 0, MSP_DIRECTION_REQUEST);

        update_read(&serialInstance);

        mspProcess(&mspPort);
    }
}

