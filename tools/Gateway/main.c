#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "io/serial.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "common/debug.h"

static serialPort_t serialInstance;
static mspPort_t mspPort;

#define BUFFER_SIZE 256
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

int fd; /* File descriptor for the port */

static void serial_writeCallback(void) {
    // nothing to transmit
    if (serialInstance.txBufferHead == serialInstance.txBufferTail) {
        return;
    }
    if (serialInstance.txBufferHead > serialInstance.txBufferTail) {
        if (write(fd, &serialInstance.txBuffer[serialInstance.txBufferTail], serialInstance.txBufferHead - serialInstance.txBufferTail)) {
            serialInstance.txBufferTail = serialInstance.txBufferHead;
        } else {
            fputs("write() failed!\n", stderr);
        }

    } else {
        if (write(fd, &serialInstance.txBuffer[serialInstance.txBufferTail], serialInstance.txBufferSize - serialInstance.txBufferTail)) {
            serialInstance.txBufferTail = 0;
        } else {
            fputs("write() failed!\n", stderr);
        }
    }
}

char buffer[256];
static void serial_read(void) {

    uint32_t bytesUsed;

    if (serialInstance.rxBufferHead >= serialInstance.rxBufferTail) {
        bytesUsed = serialInstance.rxBufferHead - serialInstance.rxBufferTail;
    } else {
        bytesUsed = serialInstance.rxBufferSize + serialInstance.rxBufferHead - serialInstance.rxBufferTail;
    }

    uint32_t free = (serialInstance.rxBufferSize - 1) - bytesUsed;

    int rdlen = read(fd, buffer, free); //read data from serial line into buffer
    for (int i = 0; i < rdlen; i++) {
        uint8_t data = buffer[i];
        //printf("%c", data);
        serialInstance.rxBuffer[serialInstance.rxBufferHead] = data;
        if (serialInstance.rxBufferHead + 1 >= serialInstance.rxBufferSize) {
            serialInstance.rxBufferHead = 0;
        } else {
            serialInstance.rxBufferHead++;
        }
    }

}

static void debugStdout(const uint8_t* data, uint16_t len) {
    fwrite(data, 1, len, stdout);
}

static int set_interface_attribs(int fdd, int speed, int parity) {
    struct termios tty;
    if (tcgetattr(fdd, &tty) != 0) {
        printf("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN] = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
                                     // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fdd, TCSANOW, &tty) != 0) {
        return -1;
    }
    return 0;
}

static void set_blocking(int fdd, int should_block) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fdd, &tty) != 0) {
        printf("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fdd, TCSANOW, &tty) != 0)
        printf("error %d setting term attributes", errno);
}

static void serial_initialize(void) {
    serialInstance.txBuffer = txBuffer;
    serialInstance.txBufferHead = 0;
    serialInstance.txBufferTail = 0;
    serialInstance.txBufferSize = BUFFER_SIZE;
    serialInstance.rxBuffer = rxBuffer;

    serialInstance.rxBufferHead = 0;
    serialInstance.rxBufferTail = 0;
    serialInstance.rxBufferSize = BUFFER_SIZE;
    serialInstance.triggerWrite = serial_writeCallback;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open_port: Unable to open");
        return;
    } else {
        set_interface_attribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking(fd, 0);
    }
    //todo dev zeug
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

int main(void) {
    initDebug(&debugStdout);
    serial_initialize();
    mspPort.mspProcessCommandFnPtr = &mspFcProcessCommand;
    mspPort.mspProcessReplyFnPtr = &mspFcProcessReply;
    mspInit(&mspPort, &serialInstance);

    while (1) {
        mspSerialPush(&mspPort, MSP_API_VERSION, 0, 0, MSP_DIRECTION_REQUEST);
        serial_read();
        mspProcess(&mspPort);
    }
}

