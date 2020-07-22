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
static double simRate = 1.0;

// buffer for serial interface
#define BUFFER_SIZE 1400
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

/***************************************************************
 *
 *              local Functions
 *
 ***************************************************************/

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
    serialInstance.serialWrite = &tcp_serialWrite;
    serialInstance.serialRead = &tcp_serialRead;
    serialInstance.serialTotalRxWaiting = &tcp_serialTotalRxWaiting;
    serialInstance.serialTotalTxFree = &tcp_serialTotalTxFree;
    serialInstance.isSerialTransmitBufferEmpty = &tcp_isSerialTransmitBufferEmpty;
    serialInstance.beginWrite = &tcp_beginWrite;
    serialInstance.endWrite = &tcp_endWrite;
    tcp_serial_initialize(&serialInstance);

    initDebug(&debugSerial);
}

void msp_initialize(void) {
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspPort.mspProcessReplyFnPtr = &mspFcProcessReply;
    mspInit(&mspPort, &serialInstance);
    initMspDebugPort(&mspPort);
    //initDebug(&mspDebugData);
}

static int64_t nanos64_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1e9 + ts.tv_nsec) - (start_time.tv_sec * 1e9 + start_time.tv_nsec);
}
//static uint64_t micros64_real() {
//    struct timespec ts;
//    clock_gettime(CLOCK_MONOTONIC, &ts);
//    return 1.0e6 * ((ts.tv_sec + (ts.tv_nsec * 1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec * 1.0e-9)));
//}
//
//static uint64_t millis64_real() {
//    struct timespec ts;
//    clock_gettime(CLOCK_MONOTONIC, &ts);
//    return 1.0e3 * ((ts.tv_sec + (ts.tv_nsec * 1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec * 1.0e-9)));
//}

static uint64_t micros64(void) {
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out * 1e-3;
//    return micros64_real();
}
static uint64_t millis64(void) {
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out*1e-6;
//    return millis64_real();
}

static uint32_t sitl_micros(void) {
    return micros64() & 0xFFFFFFFF;
}

static uint32_t sitl_millis(void) {
    return millis64() & 0xFFFFFFFF;
}

void platform_initialize(void) {
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    SystemCoreClock = 500 * 1e6; // fake 500MHz
    printf("[system]Init...\n");

    initTime(&sitl_millis, &sitl_micros);
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

