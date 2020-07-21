#include <math.h>

// driver includes
#include "asf.h"
#include "clock_support.h"
#include "spi_support.h"
#include "bmg160_support.h"
#include "bma2x2_support.h"
#include "tc_support.h"
#include "drivers/SAMD20J18/serial.h"

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
static accDev_t accDev;
static gyroDev_t gyroDev;

// buffer for serial interface
#define BUFFER_SIZE 256
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
static void samd20j18_serial_writeCallback(void) {
    // nothing to transmit
    if (serialInstance.txBufferHead == serialInstance.txBufferTail) {
        return;
    }
    if (serialInstance.txBufferHead > serialInstance.txBufferTail) {
        if (samd20j18_serial_write(&serialInstance.txBuffer[serialInstance.txBufferTail], serialInstance.txBufferHead - serialInstance.txBufferTail)) {
            serialInstance.txBufferTail = serialInstance.txBufferHead;
        }

    } else {
        if (samd20j18_serial_write(&serialInstance.txBuffer[serialInstance.txBufferTail], serialInstance.txBufferSize - serialInstance.txBufferTail)) {
            serialInstance.txBufferTail = 0;
        }
    }
}

/**
 * callback function called when data on the serial interface arrives
 * @param data
 */
static void samd20j18_serial_readCallback(uint8_t data) {
    serialInstance.rxBuffer[serialInstance.rxBufferHead] = data;
    if (serialInstance.rxBufferHead + 1 >= serialInstance.rxBufferSize) {
        serialInstance.rxBufferHead = 0;
    } else {
        serialInstance.rxBufferHead++;
    }
}

//static void debugSerial(const uint8_t* data, uint16_t len) {
//    serialWriteBuf(&serialInstance, data, len);
//}

/**
 * generate response for command requestet on msp
 * @param cmdMSP
 * @param dst
 * @return
 */
static bool mspProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst) {
    switch (cmdMSP) {
    case MSP_BATTERY_CONFIG: //TODO
        sbufWriteU8(dst, (330 + 5) / 10);
        sbufWriteU8(dst, (430 + 5) / 10);
        sbufWriteU8(dst, (350 + 5) / 10);
        sbufWriteU16(dst, 2500);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 330);
        sbufWriteU16(dst, 430);
        sbufWriteU16(dst, 350);
        break;
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;
    case MSP_FC_VARIANT:
        sbufWriteString(dst, FC_VARIANT);
        break;
    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO: { //TODO check info
        sbufWriteData(dst, BOARD_IDENTIFIER, 4);
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection
        sbufWriteU8(dst, 0);  // 0 == FC
        sbufWriteU8(dst, 0); // Target capabilities
        sbufWriteU8(dst, 4); // Name length
        sbufWriteData(dst, "TEST", 4);
        sbufWriteU8(dst, 0); //USE_BOARD_INFO
        sbufWriteU8(dst, 0); //USE_BOARD_INFO
        uint8_t emptySignature[32];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
        sbufWriteU8(dst, 255);
        break;
    }

    case MSP_BUILD_INFO: {
        const char * const shortGitRevision = "NO INFO";
        const char * const buildDate = __DATE__;
        const char * const buildTime = __TIME__;
        sbufWriteData(dst, buildDate, 11);
        sbufWriteData(dst, buildTime, 8);
        sbufWriteData(dst, shortGitRevision, 7);
        break;
    }
    case MSP_ANALOG:
        sbufWriteU8(dst, 3);
        sbufWriteU16(dst, 0); // milliamp hours drawn from battery
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0); // send current in 0.01 A steps, range is -320A to 320A
        sbufWriteU16(dst, 0);
        break;
    case MSP_DEBUG: {
        int16_t debug[4]; //todo
        for (int i = 0; i < 4; i++) {
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        }
        break;
    }
    case MSP_DEBUGMSG:
        break;
    case MSP_UID:
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 1);
        sbufWriteU32(dst, 2);
        break;
    case MSP_FEATURE_CONFIG:
        sbufWriteU32(dst, 0); //Todo
        break;
    case MSP_RAW_IMU: {
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, lrintf(accDev.ADCRaw[i]*100*accDev.scale));
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, lrintf((float)gyroDev.ADCRaw[i]*100*gyroDev.scale));
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, 100);
        }
        break;
    }
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

static bool bma280Read(accDev_t *acc) {

    struct bma2x2_accel_data rawData;
    if (bma2x2_read_accel_xyz(&rawData) != 0) {
        return false;
    }
    acc->ADCRaw[X] = rawData.x;
    acc->ADCRaw[Y] = rawData.y;
    acc->ADCRaw[Z] = rawData.z;

    return true;
}

static void accInit(void) {
    bma_init();
    //Normalizing Factor to 1G at +-8 with +-2^13 = 8192/8 = 1024
    bma2x2_set_range(BMA2x2_RANGE_8G);
    bma2x2_set_bw(BMA2x2_BW_500HZ);
    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    accDev.readFn = bma280Read;
    accDev.scale = 1.0f / 1024.0f;
}
static bool bmg160Read(gyroDev_t *gyro) {

    struct bmg160_data_t rawData;
    if (bmg160_get_data_XYZ(&rawData) != 0) {
        return false;
    }
    gyro->ADCRaw[X] = rawData.datax;
    gyro->ADCRaw[Y] = rawData.datay;
    gyro->ADCRaw[Z] = rawData.dataz;

    return true;
}
static void gyroInit(void) {
    bmg_init();
    bmg160_set_range_reg(0x00);
    bmg160_set_bw(C_BMG160_BW_116HZ_U8X);
    bmg160_set_power_mode(BMG160_MODE_NORMAL);
    gyroDev.readFn = bmg160Read;
    // 16.4 dps/lsb scalefactor
    gyroDev.scale = 1.0f / 16.4f;
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
    serialInstance.triggerWrite = samd20j18_serial_writeCallback;
    samd20j18_serial_initialize(&samd20j18_serial_writeCallback, &samd20j18_serial_readCallback);
    //initDebug(&debugSerial);
}

void msp_initialize(void) {
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspPort.mspProcessReplyFnPtr = &mspFcProcessReply;
    mspInit(&mspPort, &serialInstance);
    initMspDebugPort(&mspPort);
    initDebug(&mspDebugData);
}

void platform_initialize(void) {
    system_init();
    clock_initialize();
    tc_initialize();
    initTime(&millis_samd20j18, &micros_samd20j18);
}

void interrupt_enable(void) {
    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
}

void processMSP(void) {
    mspProcess(&mspPort);
}

void sensor_initialize(void) {
    spi_initialize();
    accInit();
    gyroInit();
}

void sensor_read(void) {
    accDev.readFn(&accDev);
    gyroDev.readFn(&gyroDev);
}

