#pragma once

#include "global.h"
#include "common/streambuf.h"
#include "common/time.h"

#define MSP_PORT_INBUF_SIZE 256
#define MSP_PORT_OUTBUF_SIZE 256

// return positive for ACK, negative on error, zero for no reply
typedef enum {
    MSP_RESULT_ACK = 1, MSP_RESULT_ERROR = -1, MSP_RESULT_NO_REPLY = 0, MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
} mspResult_e;

typedef enum {
    MSP_DIRECTION_REPLY = 0, MSP_DIRECTION_REQUEST = 1
} mspDirection_e;

typedef enum {
    MSP_IDLE, MSP_HEADER_START, MSP_HEADER_M,

    MSP_HEADER_V1, MSP_PAYLOAD_V1, MSP_CHECKSUM_V1,

    MSP_COMMAND_RECEIVED
} mspState_e;

typedef enum {
    MSP_PACKET_COMMAND, MSP_PACKET_REPLY
} mspPacketType_e;

typedef struct mspPacket_s {
    sbuf_t buf;
    int16_t cmd;
    int16_t result;
    uint8_t direction;
} mspPacket_t;


typedef struct mspPort_s {
    struct serialPort_s *port;
    timeMs_t lastActivityMs;
    mspState_e c_state;
    mspPacketType_e packetType;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    uint8_t cmdMSP;
    uint_fast16_t offset;
    uint_fast16_t dataSize;
    uint8_t checksum;
    mspResult_e (*mspProcessCommandFnPtr)(mspPacket_t *cmd, mspPacket_t *reply);
    void (*mspProcessReplyFnPtr)(mspPacket_t *cmd);
} mspPort_t;



void mspDebugData(const uint8_t* data, uint16_t len);
void mspWriteBlackBoxData(const uint8_t* data, uint16_t len);
void initMspDebugPort(mspPort_t *mspPort);
void mspInit(mspPort_t *mspPort, struct serialPort_s *serialPort);

void mspProcess(mspPort_t *port);
int mspSerialPush(mspPort_t *mspPort, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction);
