
#pragma once

#include "common/streambuf.h"

#define MSP_PORT_INBUF_SIZE 192
#define MSP_PORT_OUTBUF_SIZE 256


typedef enum {
    MSP_V1          = 0,
    MSP_V2_OVER_V1  = 1,
    MSP_V2_NATIVE   = 2,
    MSP_VERSION_COUNT
} mspVersion_e;

typedef struct mspPort_s mspPort_t;

struct mspPort_s {
    mspState_e c_state;
    u32_t lastActivityMs;
    uint16_t cmdMSP;
    ringbuffer_t *rxBuffer;
    ringbuffer_t *txBuffer;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    uint_fast16_t offset;
    uint_fast16_t dataSize;
    uint8_t checksum1;
    mspVersion_e mspVersion;
    mspPacketType_e packetType;
    mspPort_t *passthroughPort;
};
