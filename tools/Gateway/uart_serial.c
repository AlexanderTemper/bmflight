#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "uart_serial.h"

static int fd; /* File descriptor for the port */
static uint8_t buffer[2048];

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

    cfmakeraw(&tty); //Important so the terminal dont interpreate the data!!
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

void update_read(serialPort_t *instance) {

    uint32_t bytesUsed;

    if (instance->rxBufferHead >= instance->rxBufferTail) {
        bytesUsed = instance->rxBufferHead - instance->rxBufferTail;
    } else {
        bytesUsed = instance->rxBufferSize + instance->rxBufferHead - instance->rxBufferTail;
    }

    uint32_t free = (instance->rxBufferSize - 1) - bytesUsed;

    int rdlen = read(fd, buffer, free); //read data from serial line into buffer
    for (int i = 0; i < rdlen; i++) {
        uint8_t data = buffer[i];
        //printf(" %c[%d],", data, data);
        instance->rxBuffer[instance->rxBufferHead] = data;
        if (instance->rxBufferHead + 1 >= instance->rxBufferSize) {
            instance->rxBufferHead = 0;
        } else {
            instance->rxBufferHead++;
        }
    }

}

void uart_serialWrite(serialPort_t *instance, uint8_t ch) {
    //pthread_mutex_lock(&tcpSerialPort.txLock);

    instance->txBuffer[instance->txBufferHead] = ch;
    if (instance->txBufferHead + 1 >= instance->txBufferSize) {
        instance->txBufferHead = 0;
    } else {
        instance->txBufferHead++;
    }

    if (instance->txBufferHead < instance->txBufferTail) {
        // send data till end of buffer
        int chunk = instance->txBufferSize - instance->txBufferTail;
        write(fd, &instance->txBuffer[instance->txBufferTail], chunk);
        instance->txBufferTail = 0;
    }
    int chunk = instance->txBufferHead - instance->txBufferTail;
    if (chunk) {
        write(fd, &instance->txBuffer[instance->txBufferTail], chunk);
    }
    instance->txBufferTail = instance->txBufferHead;

    //  pthread_mutex_unlock(&tcpSerialPort.txLock);
}

uint8_t uart_serialRead(serialPort_t *instance) {

    uint8_t ch;
    //pthread_mutex_lock(&tcpSerialPort.rxLock);
    ch = instance->rxBuffer[instance->rxBufferTail];
    if (instance->rxBufferTail + 1 >= instance->rxBufferSize) {
        instance->rxBufferTail = 0;
    } else {
        instance->rxBufferTail++;
    }
    // pthread_mutex_unlock(&tcpSerialPort.rxLock);

    return ch;

}

uint32_t uart_serialTotalRxWaiting(const serialPort_t *instance) {
    int32_t count;
    // pthread_mutex_lock(&tcpSerialPort.rxLock);
    if (instance->rxBufferHead >= instance->rxBufferTail) {
        count = instance->rxBufferHead - instance->rxBufferTail;
    } else {
        count = instance->rxBufferSize + instance->rxBufferHead - instance->rxBufferTail;
    }
    // pthread_mutex_unlock(&tcpSerialPort.rxLock);

    return count;
}

uint32_t uart_serialTotalTxFree(const serialPort_t *instance) {
    uint32_t bytesUsed;
    //  pthread_mutex_lock(&tcpSerialPort.txLock);
    if (instance->txBufferHead >= instance->txBufferTail) {
        bytesUsed = instance->txBufferHead - instance->txBufferTail;
    } else {
        bytesUsed = instance->txBufferSize + instance->txBufferHead - instance->txBufferTail;
    }
    uint32_t bytesFree = (instance->txBufferSize - 1) - bytesUsed;
    //  pthread_mutex_unlock(&tcpSerialPort.txLock);
    return bytesFree;
}

bool uart_isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    // pthread_mutex_lock(&tcpSerialPort.txLock);
    bool isEmpty = instance->txBufferTail == instance->txBufferHead;
    //  pthread_mutex_unlock(&tcpSerialPort.txLock);
    return isEmpty;
}

void uart_beginWrite(serialPort_t *instance) {

}
void uart_endWrite(serialPort_t *instance) {

}

bool uart_serial_initialize(serialPort_t *instance, const char *devName) {
    fd = open(devName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open_port: Unable to open");
        return false;
    } else {
        set_interface_attribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking(fd, 0);
    }
    return true;
}

