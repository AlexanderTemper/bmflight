// driver includes
#include "asf.h"
#include "clock_support.h"
#include "spi_support.h"
#include "bmg160_support.h"
#include "bma2x2_support.h"
#include "tc_support.h"
#include "usart_support.h"

// common includes
#include "platform.h"
#include "fc/fc.h"
#include "common/debug.h"
#include "common/time.h"
#include "io/serial.h"
#include "msp/msp.h"
#include "msp/msp_commands.h"
#include "sensor/sensor.h"

// Variables
static serialPort_t serialInstance;
static mspPort_t mspPort;
static sensors_t sensors;
// buffer for serial interface
#define BUFFER_SIZE 256
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

//static void debugSerial(const uint8_t* data, uint16_t len) {
//    serialWriteBuf(&serialInstance, data, len);
//}

static bool bma280Read(accDev_t *acc) {

    struct bma2x2_accel_data rawData;
    if (bma2x2_read_accel_xyz(&rawData) != 0) {
        return false;
    }
    acc->ADCRaw[X] = rawData.x;
    acc->ADCRaw[Y] = rawData.y;
    acc->ADCRaw[Z] = rawData.z;
    acc->lastReadTime = micros();
    return true;
}

static void accInit(void) {
    bma_init();
    //Normalizing Factor to 1G at +-8 with +-2^13 = 8192/8 = 1024
    bma2x2_set_range(BMA2x2_RANGE_8G);
    bma2x2_set_bw(BMA2x2_BW_500HZ);
    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    sensors.acc.readFn = bma280Read;
    sensors.acc.scale = 1.0f / 1024.0f;
}
static bool bmg160Read(gyroDev_t *gyro) {

    struct bmg160_data_t rawData;
    if (bmg160_get_data_XYZ(&rawData) != 0) {
        return false;
    }
    gyro->ADCRaw[X] = rawData.datax;
    gyro->ADCRaw[Y] = rawData.datay;
    gyro->ADCRaw[Z] = rawData.dataz;
    gyro->lastReadTime = micros();

    return true;
}
static void gyroInit(void) {
    bmg_init();
    bmg160_set_range_reg(0x00);
    bmg160_set_bw(C_BMG160_BW_116HZ_U8X);
    bmg160_set_power_mode(BMG160_MODE_NORMAL);
    sensors.gyro.readFn = bmg160Read;
    // 16.4 dps/lsb scalefactor
    sensors.gyro.scale = 1.0f / 16.3835f;
}
inline static void bmf055_delayNanoSeconds(timeUs_t nsec) {

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
    serialInstance.serialWrite = &samd20j18_serialWrite;
    serialInstance.serialRead = &samd20j18_serialRead;
    serialInstance.serialTotalRxWaiting = &samd20j18_serialTotalRxWaiting;
    serialInstance.serialTotalTxFree = &samd20j18_serialTotalTxFree;
    serialInstance.isSerialTransmitBufferEmpty = &samd20j18_isSerialTransmitBufferEmpty;
    serialInstance.beginWrite = &samd20j18_beginWrite;
    serialInstance.endWrite = &samd20j18_endWrite;
    samd20j18_serial_initialize(&serialInstance);
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
    initTime(&millis_samd20j18, &micros_samd20j18, &bmf055_delayNanoSeconds);
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
    InitSonsors(&sensors);
}

