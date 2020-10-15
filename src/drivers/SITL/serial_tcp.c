#include "drivers/SITL/serial_tcp.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

static serialPort_t *serialPort;
static tcpPort_t *tcpSerialPort;

static int write_tcp_imp(uint8_t *data, int len) {
//    char *buf = data;
//    for (int i = 0; i < len; i++) {
//        printf("send :%c [0x%x]\n", *buf, *buf);
//        buf++;
//    }
    int status = send(tcpSerialPort->connfd, (const void *) data, len, MSG_NOSIGNAL);
    if (status <= 0) {
        tcpSerialPort->connected = false;
        close(tcpSerialPort->connfd);
        printf("--- MSP close connection (no write)\n");
    }
    return status;
}

static void write_tcp(void) {
    serialPort_t *instance = serialPort;

    if (!tcpSerialPort->connected) {
        instance->txBufferTail = instance->txBufferHead;
        return;
    }

    pthread_mutex_lock(&tcpSerialPort->txLock);
    if (instance->txBufferHead < instance->txBufferTail) {
        // send data till end of buffer
        int chunk = instance->txBufferSize - instance->txBufferTail;
        write_tcp_imp(&instance->txBuffer[instance->txBufferTail], chunk);
        //Todo check if send == chunk
        instance->txBufferTail = 0;
    }
    int chunk = instance->txBufferHead - instance->txBufferTail;
    if (chunk) {
        write_tcp_imp(&instance->txBuffer[instance->txBufferTail], chunk);
        //Todo check if send == chunk
    }

    instance->txBufferTail = instance->txBufferHead;

    pthread_mutex_unlock(&tcpSerialPort->txLock);
}

static void read_tcp(void) {
    if (!tcpSerialPort->connected) {
        return;
    }
    uint32_t bytesUsed;

    pthread_mutex_lock(&tcpSerialPort->rxLock);

    if (serialPort->rxBufferHead >= serialPort->rxBufferTail) {
        bytesUsed = serialPort->rxBufferHead - serialPort->rxBufferTail;
    } else {
        bytesUsed = serialPort->rxBufferSize + serialPort->rxBufferHead - serialPort->rxBufferTail;
    }
    uint32_t bytesFree = (serialPort->rxBufferSize - 1) - bytesUsed;

    int result = 0;
    while (bytesFree > 0) {
        char ch;
        result = read(tcpSerialPort->connfd, &ch, 1);
        if (result == 0) {
            tcpSerialPort->connected = false;
            close(tcpSerialPort->connfd);
            printf("--- MSP close connection (no read)\n");
        }
        //printf("result %d \n", result);
        if (result != 1) {
            break;
        }

        //printf("Read %c [0x%x]\n", ch, ch);
        serialPort->rxBuffer[serialPort->rxBufferHead] = ch;
        if (serialPort->rxBufferHead + 1 >= serialPort->rxBufferSize) {
            serialPort->rxBufferHead = 0;
        } else {
            serialPort->rxBufferHead++;
        }
        bytesFree--;
    }
    pthread_mutex_unlock(&tcpSerialPort->rxLock);
}

void tcp_serialWrite(serialPort_t *instance, uint8_t ch) {
    pthread_mutex_lock(&tcpSerialPort->txLock);
    instance->txBuffer[instance->txBufferHead] = ch;
    if (instance->txBufferHead + 1 >= instance->txBufferSize) {
        instance->txBufferHead = 0;
    } else {
        instance->txBufferHead++;
    }
    pthread_mutex_unlock(&tcpSerialPort->txLock);
    write_tcp();
}

uint8_t tcp_serialRead(serialPort_t *instance) {
    read_tcp();
    uint8_t ch;
    pthread_mutex_lock(&tcpSerialPort->rxLock);
    ch = instance->rxBuffer[instance->rxBufferTail];
    if (instance->rxBufferTail + 1 >= instance->rxBufferSize) {
        instance->rxBufferTail = 0;
    } else {
        instance->rxBufferTail++;
    }
    pthread_mutex_unlock(&tcpSerialPort->rxLock);

    return ch;
}

uint32_t tcp_serialTotalRxWaiting(const serialPort_t *instance) {
    read_tcp();
    int32_t count;
    pthread_mutex_lock(&tcpSerialPort->rxLock);
    if (instance->rxBufferHead >= instance->rxBufferTail) {
        count = instance->rxBufferHead - instance->rxBufferTail;
    } else {
        count = instance->rxBufferSize + instance->rxBufferHead - instance->rxBufferTail;
    }
    pthread_mutex_unlock(&tcpSerialPort->rxLock);

    return count;
}

uint32_t tcp_serialTotalTxFree(const serialPort_t *instance) {
    uint32_t bytesUsed;
    pthread_mutex_lock(&tcpSerialPort->txLock);
    if (instance->txBufferHead >= instance->txBufferTail) {
        bytesUsed = instance->txBufferHead - instance->txBufferTail;
    } else {
        bytesUsed = instance->txBufferSize + instance->txBufferHead - instance->txBufferTail;
    }
    uint32_t bytesFree = (instance->txBufferSize - 1) - bytesUsed;
    pthread_mutex_unlock(&tcpSerialPort->txLock);
    return bytesFree;
}

bool tcp_isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    pthread_mutex_lock(&tcpSerialPort->txLock);
    bool isEmpty = instance->txBufferTail == instance->txBufferHead;
    pthread_mutex_unlock(&tcpSerialPort->txLock);
    return isEmpty;
}

void tcp_beginWrite(serialPort_t *instance) {

}

void tcp_endWrite(serialPort_t *instance) {

}

void tcp_initialize_client(tcpPort_t *s, const char * address) {

    if (pthread_mutex_init(&s->txLock, NULL) != 0) {
        fprintf(stderr, "TX mutex init failed - %d\n", errno);
        exit(1);
    }
    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        fprintf(stderr, "RX mutex init failed - %d\n", errno);
        exit(1);
    }
    struct sockaddr_in serv_addr;
    if ((s->serverfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Error : Could not create socket \n");
        exit(1);
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(BASE_PORT);

    if (inet_pton(AF_INET, address, &serv_addr.sin_addr) <= 0) {
        printf("\n inet_pton error occured\n");
        exit(1);
    }

    if (connect(s->serverfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("Error: ");
        exit(1);
    }

    s->connected = true;
    s->connfd = s->serverfd;
    printf("set connection non-blocking\n");
    int status = fcntl(s->connfd, F_SETFL, fcntl(s->connfd, F_GETFL, 0) | O_NONBLOCK);

    if (status == -1) {
        perror("calling fcntl");
        // handle the error.  By the way, I've never seen fcntl fail in this way
    }
}

void tcp_initialize_server(tcpPort_t *s, const char * address) {
    if (pthread_mutex_init(&s->txLock, NULL) != 0) {
        fprintf(stderr, "TX mutex init failed - %d\n", errno);
        exit(1);
    }
    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        fprintf(stderr, "RX mutex init failed - %d\n", errno);
        exit(1);
    }
    s->serverfd = socket(AF_INET, SOCK_STREAM, 0);
    if (s->serverfd < 0) {
        perror("open stream socket");
        exit(1);
    }
    struct sockaddr_in serv_addr;
    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(address);
    serv_addr.sin_port = htons(BASE_PORT);

    int enable = 1;
    if (setsockopt(s->serverfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
    }
    if (bind(s->serverfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0) {
        //print the error message
        perror("bind failed. Error");
        exit(1);
    }
    if (listen(s->serverfd, 10) == 0) {
        printf("TCP Port Listen Listening Port %d\n", BASE_PORT);
    } else {
        perror("listen failed. Error");
        exit(1);
    }
}

void tcp_serial_initialize(serialPort_t *instance, tcpPort_t *stcpPort) {
    serialPort = instance;
    tcpSerialPort = stcpPort;
}
