#include <stdio.h>   /* Standard input/output definitions */
#include "io/serial.h"
#include "msp/msp.h"
#include "common/time.h"
#include "msp/msp_protocol.h"
#include "common/debug.h"
#include "uart_serial.h"
#include "drivers/SITL/serial_tcp.h"
#include "drivers/SITL/udplink.h"
#include "joy.h"
#include "fc/tasks.h"
#include "scheduler/scheduler.h"
#include "common/streambuf.h"

// Mode Flags
bool uart = false;
bool tcp = false;
bool joyEnabled = false;
bool simBridgeEnabled = false;

#define BLACKBOX_LOGFILE_NAME "bmflog.TXT"
static struct timespec start_time;
static serialPort_t serialInstance;
static mspPort_t mspPort;
static tcpPort_t tcpSerialPort;
static FILE *blackBoxFile;

// udp package definition
#define UDP_SIM_BRIDGE_PORT 8812
static udpLink_t fromSimLink;
static udpLink_t toSimLink;
static pthread_t udpWorker;
typedef struct {
    double timestamp;  // in seconds
} sim_Rx_packet;

typedef struct {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} att_packet;

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

static int logCounter = 0;
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
        for (int i = 0; i < 3; i++) {
            att[i] = sbufReadU16(src);
        }

        if (tcp && simBridgeEnabled) {
            att_packet attPkg;
            attPkg.roll = att[0];
            attPkg.pitch = att[1];
            attPkg.yaw = att[2];
            udpSend(&toSimLink, &attPkg, sizeof(attPkg));
            //printf("send!! %d %d %d\n", attPkg.roll, attPkg.pitch, attPkg.yaw);
            return;
        }
        printf("\n get ATT %d %d %d\n", att[X], att[Y], att[Z]);

        break;
    }
    case MSP_BLACKBOX_DATA: {
        fwrite(src->ptr, 1, sbufBytesRemaining(src), blackBoxFile);
        logCounter++;
        printf("%d\r", logCounter);
        fflush(stdout);
        fclose(blackBoxFile);
        blackBoxFile = fopen(BLACKBOX_LOGFILE_NAME, "a+");
        if (blackBoxFile == NULL) {
            fprintf(stderr, "[BLACKBOX] failed to create '%s'\n", BLACKBOX_LOGFILE_NAME);
        }
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
    case MSP_BLACKBOX_START: {
        printf("Logger started\n");
        fflush(stdout);
        break;
    }
    case MSP_BLACKBOX_STOP: {
        printf("Logger stopped\n");
        break;
    }
    default:
        printf("Unknown msp package");
        break;
    }
}

int armedFake = true;
int wobbleFake = 1000;
static void taskJoy(timeUs_t currentTimeUs) {
    readJoy();
    uint8_t data[12];
    sbuf_t buf;
    sbufInit(&buf, &data[0], &data[11]);

//    rx_joy.arm = 1000;
//    rx_joy.roll = 1500;
//    rx_joy.pitch = 1500;
//    rx_joy.yaw = 1500;
//    rx_joy.throttle = (wobbleFake += 10) % 1000 + 1000;
//    rx_joy.arm = armedFake ? 2000 : 1000;

    sbufWriteU16(&buf, rx_joy.roll);
    sbufWriteU16(&buf, rx_joy.pitch);
    sbufWriteU16(&buf, rx_joy.yaw);
    sbufWriteU16(&buf, rx_joy.throttle);
    sbufWriteU16(&buf, rx_joy.arm);
    sbufWriteU16(&buf, 1);  // Todo wenn hier 5 gehts ned was macht liux da

    sbufSwitchToReader(&buf, &data[0]);
    //printf("roll %6d, pitch %6d, yaw %6d, thrust %6d, arm %d \n", rx_joy.roll, rx_joy.pitch, rx_joy.yaw, rx_joy.throttle, rx_joy.arm);
    mspSerialPush(&mspPort, MSP_SET_RAW_RC, data, sbufBytesRemaining(&buf), MSP_DIRECTION_REQUEST);
}

static void taskSystem(timeUs_t currentTimeUs) {
    //mspSerialPush(&mspPort, MSP_ATTITUDE, 0, 0, MSP_DIRECTION_REQUEST);
}

static void taskLogger(timeUs_t currentTimeUs) {
    //mspSerialPush(&mspPort, MSP_BLACKBOX_STOP, 0, 0, MSP_DIRECTION_REQUEST);
}
static void taskHandleSerial(timeUs_t currentTimeUs) {
    if (uart) {
        update_read(&serialInstance);
    }
    mspProcess(&mspPort);
}

static void taskLoop(timeUs_t currentTimeUs) {

}

/**
 * reqeust attitude form fc
 * @param currentTimeUs
 */
static void taskAtt(timeUs_t currentTimeUs) {
    mspSerialPush(&mspPort, MSP_ATTITUDE, 0, 0, MSP_DIRECTION_REQUEST); //request Attitude
}

static task_t tasks[TASK_COUNT] = {
    [TASK_LOOP] = { //Needed RealTime Task (in firmware main gyro looptime)
        .taskName = "TASK_LOOP",
        .taskFunc = taskLoop,
        .staticPriority = 200,
        .desiredPeriodUs = 100000000, },
    [TASK_ATTITUDE] = {
        .taskName = "TASK_ATT",
        .taskFunc = taskAtt,
        .staticPriority = 1,
        .desiredPeriodUs = TASK_PERIOD_HZ(50), },
    [TASK_SERIAL] = {
        .taskName = "TASK_SERIAL",
        .taskFunc = taskHandleSerial,
        .staticPriority = 4,
        .desiredPeriodUs = TASK_PERIOD_HZ(10000), }, //Task needs to be run with high rate to be able process MSP_BLACKBOX_DATA in time
    [TASK_RX] = {
        .taskName = "TASK_RX",
        .taskFunc = taskJoy,
        .staticPriority = 1,
        .desiredPeriodUs = TASK_PERIOD_HZ(100), },
    [TASK_DEBUG] = {
        .taskName = "TASK_DEBUG",
        .taskFunc = taskLogger,
        .staticPriority = 1,
        .desiredPeriodUs = 15000000, }, //60sec
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

const char usage[] = "-------- Usage --------\n\n  gateway [uart] [devName]"
        " or gateway [tcp] [address]\n\n"
        "  -simBridge .... enable bride to simulation\n"
        "  -joyEnable .... enable joy stick input\n\n\n";

static int parseArgs(int argc, char *argv[]) {

    char * address;
    for (int i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "-joyEnable")) {
            joyEnabled = true;
        }
        if (!strcmp(argv[i], "-simBridge")) {
            simBridgeEnabled = true;
        }

        if (!strcmp(argv[i], "uart")) {
            uart = true;
            if (i >= (argc - 1)) {
                printf("!! please define device\n\n");
                return -1;
            }
            if (!initialize_uart_serial(argv[i + 1])) {
                return -1;
            }
        }
        if (!strcmp(argv[i], "tcp")) {
            tcp = true;
            if (i >= (argc - 1)) {
                printf("!! please define address\n\n");
                return -1;
            }
            tcp_initialize_client(&tcpSerialPort, argv[i + 1]); //setup tcp Port
            initialize_tcp_serial();
            printf("--->TCP connected to %s:%d\n", argv[i + 1], BASE_PORT);
        }
    }

    return 0;
}

/**
 * thread for handling udp packages
 * @param data
 */
static void* udpThread(void* data) {
    (void) (data);
    while (tcp && simBridgeEnabled) {
        sim_Rx_packet simRxPkt;
        int n = udpRecv(&fromSimLink, &simRxPkt, sizeof(simRxPkt), 1);
        if (n == sizeof(simRxPkt)) {
            printf("got rx paket\n\n");
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}

int main(int argc, char *argv[]) {
    initDebug(&debugStdout);
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    initTime(&sitl_millis, &sitl_micros, &sitl_delayNanoSeconds);

    if (parseArgs(argc, argv) < 0) {
        printf(usage);
        return -1;
    }

    if (tcp && simBridgeEnabled) {
        int ret = udpInit(&fromSimLink, NULL, UDP_SIM_BRIDGE_PORT, true);
        if (ret != 0) {
            printf("simBridge error!\n");
            return -1;
        }
        ret = udpInit(&toSimLink, NULL, UDP_SIM_BRIDGE_PORT + 1, false);
        if (ret != 0) {
            printf("simBridge error!\n");
            return -1;
        }
        printf("--->simBridge is enabled\n");
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

    int js = -1;
    if (joyEnabled) {
        int js = initJoy("/dev/input/js0");

        if (js == -1) {
            perror("Joy not connected");
            return -1;
        } else {
            setTaskEnabled(TASK_RX, true);
        }
    }

    if (tcp && simBridgeEnabled) {
        int ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
        if (ret != 0) {
            printf("Create udpWorker error!\n");
            return -1;
        }

        setTaskEnabled(TASK_ATTITUDE, true);
        printf("--->Task Attitude enabled\n");
    }

    setTaskEnabled(TASK_SERIAL, true);
    setTaskEnabled(TASK_LOOP, true);
    setTaskEnabled(TASK_DEBUG, true);

    while (1) {
        scheduler();
        delayNanoSeconds(50);

    }

    if (js != -1) {
        close(js);
    }

    return 0;
}

