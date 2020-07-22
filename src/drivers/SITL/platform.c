#include <math.h>

// driver includes
#include "drivers/SITL/serial_tcp.h"
#include <time.h>
#include <stdio.h>
// common includes
#include "platform.h"
#include "fc.h"
#include "common/debug.h"
#include "common/time.h"
#include "io/serial.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "sensor/sensor.h"

// Variables
static serialPort_t serialInstance;
static mspPort_t mspPort;

static struct timespec start_time;
uint32_t SystemCoreClock;

// buffer for serial interface
#define BUFFER_SIZE 1400
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

/***************************************************************
 *
 *              local Functions
 *
 ***************************************************************/
/**
 * callback function for writing data to the serial interface
 */
static void tcp_serial_writeCallback(void) {

    // nothing to transmit
    if (serialInstance.txBufferHead == serialInstance.txBufferTail) {
        return;
    }

    int chunk = serialInstance.txBufferSize - serialInstance.txBufferTail;
    if (serialInstance.txBufferHead < serialInstance.txBufferTail) {
        tcp_serial_write(&serialInstance.txBuffer[serialInstance.txBufferTail], chunk);
        serialInstance.txBufferTail = 0;
    }
    chunk = serialInstance.txBufferHead - serialInstance.txBufferTail;
    tcp_serial_write(&serialInstance.txBuffer[serialInstance.txBufferTail], serialInstance.txBufferSize - serialInstance.txBufferTail);

    serialInstance.txBufferTail = serialInstance.txBufferHead;
}

/**
 * callback function called when data on the serial interface arrives
 * @param data
 */
static void tcp_serial_readCallback(uint8_t data) {
    serialInstance.rxBuffer[serialInstance.rxBufferHead] = data;
    if (serialInstance.rxBufferHead + 1 >= serialInstance.rxBufferSize) {
        serialInstance.rxBufferHead = 0;
    } else {
        serialInstance.rxBufferHead++;
    }
}

static void debugSerial(const uint8_t* data, uint16_t len) {
    serialWriteBuf(&serialInstance, data, len);
}

/**
 * generate response for command requestet on msp
 * @param cmdMSP
 * @param dst
 * @return
 */
static bool mspProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst) {
    switch (cmdMSP) {

    default:
        return false;
    }
    return true;
}

/**
 * msp commands for setting values
 * @param cmdMSP
 * @param src
 * @return
 */
static mspResult_e mspProcessInCommand(uint8_t cmdMSP, sbuf_t *src) {
    //const unsigned int dataSize = sbufBytesRemaining(src);
    switch (cmdMSP) {
    case MSP_SET_RAW_RC:
        //todo
        break;
    default:
        //printf("Command not found");
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

/**
 * function called when request on msp interface is received
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

    if (mspProcessOutCommand(cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    } else {
        ret = mspProcessInCommand(cmdMSP, src);
    }
    reply->result = ret;
    return ret;
}
/**
 * function called when an reply is received on the msp interface
 */
static void mspFcProcessReply(mspPacket_t *cmd) {
    //no Master so nothing to do here
}

/***************************************************************
 *
 *             global Functions
 *
 ***************************************************************/

void serial_initialize(void) {
    serialInstance.txBuffer = txBuffer;
    serialInstance.txBufferHead = 0;
    serialInstance.txBufferTail = 0;
    serialInstance.txBufferSize = BUFFER_SIZE;
    serialInstance.rxBuffer = rxBuffer;

    serialInstance.rxBufferHead = 0;
    serialInstance.rxBufferTail = 0;
    serialInstance.rxBufferSize = BUFFER_SIZE;
    serialInstance.triggerWrite = tcp_serial_writeCallback;
    tcp_serial_initialize(&tcp_serial_writeCallback, &tcp_serial_readCallback);

    initDebug(&debugSerial);
}

void msp_initialize(void) {
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspPort.mspProcessReplyFnPtr = &mspFcProcessReply;
    mspInit(&mspPort, &serialInstance);
    initMspDebugPort(&mspPort);
    //initDebug(&mspDebugData);
}
static uint32_t micros32_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6 * ((ts.tv_sec + (ts.tv_nsec * 1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec * 1.0e-9)));
}
static uint32_t millis32_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3 * ((ts.tv_sec + (ts.tv_nsec * 1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec * 1.0e-9)));
}
void platform_initialize(void) {
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    SystemCoreClock = 500 * 1e6; // fake 500MHz
    printf("[system]Init...\n");

    initTime(&millis32_real, &micros32_real);
}

void interrupt_enable(void) {
//    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
}

void processMSP(void) {
    mspProcess(&mspPort);
}

void sensor_initialize(void) {
//    spi_initialize();
//    accInit();
//    gyroInit();
}

void sensor_read(void) {
//    accDev.readFn(&accDev);
//    gyroDev.readFn(&gyroDev);
}

