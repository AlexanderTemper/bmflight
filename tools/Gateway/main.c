#include <stdio.h>   /* Standard input/output definitions */
#include "io/serial.h"
#include "msp/msp.h"
#include "common/time.h"
#include "msp/msp_protocol.h"
#include "common/debug.h"
#include "uart_serial.h"
#include "drivers/SITL/serial_tcp.h"
#include "joy.h"
#include "fc/tasks.h"
#include "scheduler/scheduler.h"
#include "common/streambuf.h"

#define BLACKBOX_LOGFILE_NAME "bmflog.TXT"
static struct timespec start_time;
static serialPort_t serialInstance;
static mspPort_t mspPort;
static tcpPort_t tcpSerialPort;
static FILE *blackBoxFile;
bool uart = false;

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

static void initialize_tcp_serial(void) {
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
    case MSP_ATTITUDE: {
        int16_t att[3];
        printf("\n get ATT");
        for (int i = 0; i < 3; i++) {
            att[i] = sbufReadU16(src);
        }
        printf(" %d %d %d\n", att[X], att[Y], att[Z]);
        break;
    }
    case MSP_BLACKBOX_DATA: {
        fwrite(src->ptr, 1, sbufBytesRemaining(src), blackBoxFile);
        fclose(blackBoxFile);
        blackBoxFile = fopen(BLACKBOX_LOGFILE_NAME, "a+");
        if (blackBoxFile == NULL) {
            fprintf(stderr, "[BLACKBOX] failed to create '%s'\n", BLACKBOX_LOGFILE_NAME);
        }
        //fwrite(src->ptr, 1, sbufBytesRemaining(src), stdout);
        //printf(" , ");
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
        printf("\n get AccData [%d %d %d], GyroData [%d %d %d]", acc[X], acc[Y], acc[Z], gyro[X], gyro[Y], gyro[Z]);
        break;
    }
    case MSP_SET_RAW_RC:
        break;
    case MSP_DEBUGMSG: {
        fwrite(src->ptr, 1, sbufBytesRemaining(src), stdout);
        //printf(" , ");
        break;
    }
    default:
        printf("WATT??\n");
        break;
    }
}

static void taskJoy(timeUs_t currentTimeUs) {
    readJoy();
//        for (int i = 0; i < RX_CHANL_COUNT; i++) {
//            sbufWriteU16(dst, rx->chan[i]);
//        }

    uint8_t data[12];
    sbuf_t buf;
    sbufInit(&buf, &data[0], &data[12]);

    sbufWriteU16(&buf, rx_joy.roll);
    sbufWriteU16(&buf, rx_joy.pitch);
    sbufWriteU16(&buf, rx_joy.yaw);
    sbufWriteU16(&buf, rx_joy.throttle);
    sbufWriteU16(&buf, rx_joy.arm);
    sbufWriteU16(&buf, 5);

    printf("roll %6d, pitch %6d, yaw %6d, thrust %6d, arm %d \n", rx_joy.roll, rx_joy.pitch, rx_joy.yaw, rx_joy.throttle, rx_joy.arm);

    mspSerialPush(&mspPort, MSP_SET_RAW_RC, &data[0], 12, MSP_DIRECTION_REQUEST);
}

static void taskSystem(timeUs_t currentTimeUs) {
    // mspSerialPush(&mspPort, MSP_ATTITUDE, 0, 0, MSP_DIRECTION_REQUEST);
}

static void taskLogger(timeUs_t currentTimeUs) {
   // mspSerialPush(&mspPort, MSP_BLACKBOX_STOP, 0, 0, MSP_DIRECTION_REQUEST);
}
static void taskHandleSerial(timeUs_t currentTimeUs) {
    if (uart) {
        update_read(&serialInstance);
    }
    mspProcess(&mspPort);
}

static task_t tasks[TASK_COUNT] = {
    [TASK_SERIAL] = {
        .taskName = "TASK_SERIAL",
        .taskFunc = taskHandleSerial,
        .staticPriority = 4,
        .desiredPeriodUs = TASK_PERIOD_HZ(250), },
    [TASK_RX] = {
        .taskName = "TASK_RX",
        .taskFunc = taskJoy,
        .staticPriority = 1,
        .desiredPeriodUs = TASK_PERIOD_HZ(100), },
    [TASK_DEBUG] = {
        .taskName = "TASK_DEBUG",
        .taskFunc = taskLogger,
        .staticPriority = 1,
        .desiredPeriodUs = 60000000, }, //60sec
    [TASK_SYSTEM] = {
        .taskName = "TASK_SYSTEM",
        .taskFunc = taskSystem,
        .staticPriority = 1,
        .desiredPeriodUs = TASK_PERIOD_HZ(100), }, };

/**
 * get the task with the given taskId or NULL on error
 * @param taskId
 * @return
 */
task_t *getTask(unsigned taskId) {
    return &tasks[taskId];
}
static int64_t nanos64_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1e9 + ts.tv_nsec) - (start_time.tv_sec * 1e9 + start_time.tv_nsec);
}
static uint64_t micros64_real(void) {
    return nanos64_real() * 1e-3;
//    return micros64_real();
}
static uint64_t millis64_real(void) {
    return nanos64_real() * 1e-6;
}
static uint32_t sitl_micros(void) {
    return micros64_real() & 0xFFFFFFFF;
}

static uint32_t sitl_millis(void) {
    return millis64_real() & 0xFFFFFFFF;
}

static void sitl_delayNanoSeconds(timeUs_t nsec) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = nsec * 1UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
    }
}

int main(int argc, char *argv[]) {
    initDebug(&debugStdout);
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    initTime(&sitl_millis, &sitl_micros, &sitl_delayNanoSeconds);

    if (argc == 3 && !strcmp(argv[1], "uart")) {
        printf("Start Uart Serial\n");
        if (!initialize_uart_serial(argv[2])) {
            return -1;
        }
        uart = true;

    } else if (argc == 2 && !strcmp(argv[1], "tcp")) {
        tcp_initialize_client(&tcpSerialPort); //setup tcp Port
        initialize_tcp_serial();
    } else {
        printf("-------- Usage --------\n    gateway [uart] [devName]\n    or\n    gateway [tcp]\n");
        return -1;
    }

    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspPort.mspProcessReplyFnPtr = &mspFcProcessReply;
    mspInit(&mspPort, &serialInstance);

    blackBoxFile = fopen(BLACKBOX_LOGFILE_NAME, "a+");
    if (blackBoxFile == NULL) {
        fprintf(stderr, "[BLACKBOX] failed to create '%s'\n", BLACKBOX_LOGFILE_NAME);
        return 1;
    }
    schedulerInit();

    int js = initJoy("/dev/input/js0");

    if (js == -1) {
        perror("Joy not connected");
    } else {
        setTaskEnabled(TASK_RX, true);
    }

    setTaskEnabled(TASK_SERIAL, true);
    setTaskEnabled(TASK_DEBUG, true);

    //mspSerialPush(&mspPort, MSP_BLACKBOX_START, 0, 0, MSP_DIRECTION_REQUEST); //start logging immediate
    while (1) {
        scheduler();
        delayNanoSeconds(50);
    }

    close(js);
    return 0;
}

