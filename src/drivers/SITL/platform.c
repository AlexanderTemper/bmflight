#include <math.h>
#include <stdlib.h>

// driver includes
#include "drivers/SITL/serial_tcp.h"
#include "drivers/SITL/udplink.h"
#include <time.h>
#include <stdio.h>

// common includes
#include "platform.h"
#include "fc/fc.h"
#include "common/debug.h"
#include "common/time.h"
#include "io/serial.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "sensor/sensor.h"

// Variables
static serialPort_t serialInstance;
static mspPort_t mspPort;
static tcpPort_t tcpSerialPort;

static struct timespec start_time;
uint32_t SystemCoreClock;
static double simRate = 1.0;
static pthread_t tcpWorker, udpWorker;
static udpLink_t stateLink;
static bool workerRunning = true;

// buffer for serial interface
#define BUFFER_SIZE 1400
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

// udp package definition
#define UDP_SIM_PORT 8810
typedef struct {
    double timestamp;                   // in seconds
    double imu_angular_velocity_rpy[3]; // rad/s -> range: +/- 8192; +/- 2000 deg/se
    double imu_linear_acceleration_xyz[3];    // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
    double imu_orientation_quat[4];     //w, x, y, z
    double velocity_xyz[3];             // m/s, earth frame
    double position_xyz[3];             // meters, NED from origin
} sim_packet;

/***************************************************************
 *
 *              local Functions
 *
 ***************************************************************/

static void debugSerial(const uint8_t* data, uint16_t len) {
    serialWriteBuf(&serialInstance, data, len);
}

/**
 * thread waiting for tcp connection and sending and rec data
 * @param data
 */
static void* tcpThread(void* data) {
    printf("--- MSP-Thread started!!\n");
    while (workerRunning) {
        if (!tcpSerialPort.connected) {
            printf("--- MSP wait for connection\n");
            int connfd = accept(tcpSerialPort.serverfd, (struct sockaddr*) NULL, NULL);
            int status = fcntl(connfd, F_SETFL, fcntl(connfd, F_GETFL, 0) | O_NONBLOCK);

            if (status == -1) {
                perror("calling fcntl");
                // handle the error.  By the way, I've never seen fcntl fail in this way
            }
            tcpSerialPort.connected = true;
            tcpSerialPort.connfd = connfd;
            printf("--- MSP client connected \n");
        }
        sleep(1);
    }

    printf("tcpThread end!!\n");
    return NULL;
}
/**
 * thread for handling udp packages
 * @param data
 */
static void* udpThread(void* data) {
    (void) (data);
    while (workerRunning) {
        sim_packet simPkt;
        int n = udpRecv(&stateLink, &simPkt, sizeof(sim_packet), 100);
        if (n == sizeof(sim_packet)) {
            printf("%f: [%+.4f,%+.4f,%+.4f] [%+.4f,%+.4f,%+.4f]\n", simPkt.timestamp, simPkt.imu_angular_velocity_rpy[0], simPkt.imu_angular_velocity_rpy[1],
                    simPkt.imu_angular_velocity_rpy[2], simPkt.imu_linear_acceleration_xyz[0], simPkt.imu_linear_acceleration_xyz[1],
                    simPkt.imu_linear_acceleration_xyz[2]);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}

/****************** Time **********************/
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

    return out * 1e-6;
//    return millis64_real();
}

static uint32_t sitl_micros(void) {
    return micros64() & 0xFFFFFFFF;
}

static uint32_t sitl_millis(void) {
    return millis64() & 0xFFFFFFFF;
}

static void sitl_delayNanoSeconds(timeUs_t nsec) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = nsec * 1UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
    }
}

/*********** MSP Functions *****************/

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
            sbufWriteU16(dst, 0);
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, 0);
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

    tcp_serial_initialize(&serialInstance, &tcpSerialPort);
}

void msp_initialize(void) {
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspPort.mspProcessReplyFnPtr = &mspFcProcessReply;
    mspInit(&mspPort, &serialInstance);
    initMspDebugPort(&mspPort);
    initDebug(&mspDebugData);
}

void platform_initialize(void) {
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    SystemCoreClock = 500 * 1e6; // fake 500MHz
    printf("[system]Init...\n");

    initTime(&sitl_millis, &sitl_micros, &sitl_delayNanoSeconds);
    tcp_initialize_server(&tcpSerialPort); //setup tcp Port

    int ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    ret = udpInit(&stateLink, NULL, UDP_SIM_PORT, true);
    printf("start UDP server...%d\n", ret);

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }
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

