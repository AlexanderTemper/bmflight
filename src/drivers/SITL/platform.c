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
#include "common/maths.h"
#include "io/serial.h"
#include "io/motor.h"
#include "io/pin.h"
#include "msp/msp_commands.h"
#include "sensor/sensor.h"
#include "eeprom/eeprom_emulation.h"

// Variables
static serialPort_t serialInstance;
static mspPort_t mspPort;
static tcpPort_t tcpSerialPort;

static struct timespec start_time;
uint32_t SystemCoreClock;
static pthread_t tcpWorker, udpWorker;
static udpLink_t stateLink, pwmLink;
static pthread_mutex_t udpLock;
static bool workerRunning = true;

static sensors_t sensors;

static uint8_t pins[LEDS_COUNT];

// buffer for serial interface
#define BUFFER_SIZE 256
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

sim_packet lastSimPkt;

typedef struct {
    double timestamp;                   // in seconds
    uint16_t motor_speed[4]; // [1000-2000]
} servo_packet;

static servo_packet pwmPkt;

/***************************************************************
 *
 *              local Functions
 *
 ***************************************************************/

#define EEPROM_FILENAME "EEPROM_SITL.bin"

static void sitl_read_EEPROM(config_t* config) {
    FILE *eepromFd;
    eepromFd = fopen(EEPROM_FILENAME, "r");
    if (eepromFd == NULL) {
        printf("[FLASH] no flash File found\n");
        return;
    }
    fseek(eepromFd, 0, SEEK_END);
    size_t lSize = ftell(eepromFd);
    rewind(eepromFd);
    size_t n = fread(config, sizeof(config_t), 1, eepromFd);
    if (n == 1) {
        printf("[FLASH] loaded '%s', size = %ld / %ld\n", EEPROM_FILENAME, lSize, sizeof(config_t));
    } else {
        printf("[FLASH] loaded faild '%s', size = %ld / %ld\n", EEPROM_FILENAME, lSize, sizeof(config_t));
    }
    fclose(eepromFd);
}

static void sitl_write_EEPROM(config_t* config) {
    FILE *eepromFd;
    eepromFd = fopen(EEPROM_FILENAME, "w+");
    if (eepromFd == NULL) {
        fprintf(stderr, "[FLASH] failed to create '%s'\n", EEPROM_FILENAME);
        return;
    }
    if (fwrite(config, sizeof(config_t), 1, eepromFd) != 1) {
        fprintf(stderr, "[FLASH] write failed: %s\n", strerror(errno));
        return;
    }
    fclose(eepromFd);
    printf("[FLASH] write config version %d  '%s', size =  %ld\n", config->CONFIG_VERSION, EEPROM_FILENAME, sizeof(config_t));
    sitl_read_EEPROM(config);
}

//static void debugSerial(const uint8_t* data, uint16_t len) {
//    serialWriteBuf(&serialInstance, data, len);
//}
/****************** Time **********************/
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
/**
 *
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

static void updateState(const sim_packet * pkt) {
    static double last_timestamp = 0; // in seconds
    static uint64_t last_realtime = 0; // in uS

    const uint64_t realtime_now = micros64_real();

    if (realtime_now > last_realtime + 500 * 1e3) { // 500ms timeout
        printf("Timeout Sim after %ld \n", (realtime_now - last_realtime) / 1000);
        last_timestamp = pkt->timestamp;
        last_realtime = realtime_now;
        return;
    }

    const double deltaSim = pkt->timestamp - last_timestamp;  // in seconds
    if (deltaSim < 0) { // don't use old packet
        return;
    }

//    timeUs_t sim_time_curr = pkt->timestamp * 1000000;
//    timeDelta_t sim_delta = cmpTimeUs(sim_time_curr, last_timestamp * 1000000);
//    printf("current: %d ,delta %d , hz: %f\n", sim_time_curr, sim_delta, 1000000 / (float) sim_delta);

    pthread_mutex_lock(&udpLock);
    memcpy(&lastSimPkt, pkt, sizeof(sim_packet));
    pthread_mutex_unlock(&udpLock);

    last_timestamp = pkt->timestamp;
    last_realtime = micros64_real();
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
            updateState(&simPkt);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}
static int16_t recFunction(timeUs_t currentTime) {
    static timeUs_t lastTime;
    static bool level;
    if (cmpTimeUs(currentTime, lastTime) > 10000) {
        level = !level;
        lastTime = currentTime;
    }

    if (level) {
        return 1000;
    }
    return 0;
}

static int16_t SinFunction(timeUs_t currentTime) {
#define SIN_PERIOD 50000 //120Hz
    float omega = 2 * M_PIf / SIN_PERIOD;
    int periods = currentTime / SIN_PERIOD;
    uint32_t time = currentTime - periods * SIN_PERIOD;
    return 4000 * sin_approx(omega * time);
}
int16_t accfake = -8192;
static bool acc_simRead(accDev_t *acc) {
    pthread_mutex_lock(&udpLock);

//    acc->ADCRaw[X] = SinFunction(micros()); //lastSimPkt.imu_linear_acceleration_xyz[X];
//    acc->ADCRaw[Y] = accfake++; //lastSimPkt.imu_linear_acceleration_xyz[Y];
//    acc->ADCRaw[Z] = -8192; //lastSimPkt.imu_linear_acceleration_xyz[Z];

    acc->ADCRaw[X] = lastSimPkt.imu_linear_acceleration_xyz[X];
    acc->ADCRaw[Y] = lastSimPkt.imu_linear_acceleration_xyz[Y];
    acc->ADCRaw[Z] = lastSimPkt.imu_linear_acceleration_xyz[Z];
    acc->lastReadTime = lastSimPkt.timestamp * 1000000;

    pthread_mutex_unlock(&udpLock);

    return true;
}

int16_t fake = -32767;
static bool gyro_simRead(gyroDev_t *gyro) {

    pthread_mutex_lock(&udpLock);

//    gyro->raw[X] = recFunction(micros()); //lastSimPkt.imu_angular_velocity_rpy[X];
//    gyro->raw[Y] = fake++; //lastSimPkt.imu_angular_velocity_rpy[Y];
//    gyro->raw[Z] = 0;//SinFunction(micros()); //lastSimPkt.imu_angular_velocity_rpy[Z];

    gyro->raw[X] = lastSimPkt.imu_angular_velocity_rpy[X];
    gyro->raw[Y] = lastSimPkt.imu_angular_velocity_rpy[Y];
    gyro->raw[Z] = lastSimPkt.imu_angular_velocity_rpy[Z];

    gyro->lastReadTime = lastSimPkt.timestamp * 1000000;

    pthread_mutex_unlock(&udpLock);

    return true;
}

static void motor_write_sim(motors_command_t *motors) {
    pwmPkt.motor_speed[0] = motors->value[0];
    pwmPkt.motor_speed[1] = motors->value[1];
    pwmPkt.motor_speed[2] = motors->value[2];
    pwmPkt.motor_speed[3] = motors->value[3];

    //printf("write motor %d,%d,%d,%d\n",pwmPkt.motor_speed[0],pwmPkt.motor_speed[1],pwmPkt.motor_speed[2],pwmPkt.motor_speed[3]);
    pwmPkt.timestamp = lastSimPkt.timestamp * 1000000;
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
}

static void setPin(status_leds_e pinId, bool level) {
    pins[pinId] = level;
    // printf("pin Update [%c][%c][%c]\r", pins[ARM_LED] ? 'X' : ' ', pins[CALIBRATION_LED] ? 'X' : ' ', pins[ERROR_LED] ? 'X' : ' ');
    // fflush(stdout);
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

    init_EEPROM(&sitl_read_EEPROM, &sitl_write_EEPROM);
    motorSetup(&motor_write_sim);
    printf("[system]Init...\n");

    initTime(&sitl_millis, &sitl_micros, &sitl_delayNanoSeconds);
    tcp_initialize_server(&tcpSerialPort, "127.0.0.1"); //setup tcp Port

    int ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    ret = udpInit(&stateLink, NULL, UDP_SIM_PORT, true);
    if (ret != 0) {
        printf("Cannot create stateLink socket!\n");
        exit(1);
    }
    if (pthread_mutex_init(&udpLock, NULL) != 0) {
        fprintf(stderr, "udp init failed - %d\n", errno);
        exit(1);
    }
    ret = udpInit(&pwmLink, "127.0.0.1", UDP_SIM_PORT + 1, false);
    if (ret != 0) {
        printf("Cannot create pwmLink socket!\n");
        exit(1);
    }

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }

    //init pin driver
    initStatusLed(&setPin);
}

void interrupt_enable(void) {
//    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
}

void processMSP(void) {
    mspProcess(&mspPort);
}

void sensor_initialize(void) {
    sensors.gyro.readFn = gyro_simRead;
    sensors.gyro.scale = 1.0f / 16.3835f;
    sensors.acc.readFn = acc_simRead;
    sensors.acc.scale = 1.0f / 1024.0f;
    InitSonsors(&sensors);
}
