#include "fc.h"
#include "io/serial.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"

static mspPort_t *debugPort;
void initMspDebugPort(mspPort_t *mspPort) {
    debugPort = mspPort;
}

void mspInit(mspPort_t *mspPort, struct serialPort_s *serialPort) {
    mspPort->port = serialPort;
}

/**
 * State Machine for decoding MSP
 * @param mspPort
 * @param c
 * @return
 */
static bool mspSerialProcessReceivedData(mspPort_t *mspPort, const uint8_t c) {
    switch (mspPort->c_state) {
    default:
    case MSP_IDLE:      // Waiting for '$' character
        if (c == '$') {
            mspPort->c_state = MSP_HEADER_START;
        } else {
            return false;
        }
        break;

    case MSP_HEADER_START:  // Waiting for 'M' (MSPv1 / MSPv2_over_v1) or 'X' (MSPv2 native)
        mspPort->offset = 0;
        mspPort->checksum = 0;
        switch (c) {
        case 'M':
            mspPort->c_state = MSP_HEADER_M;
            break;
        default:
            mspPort->c_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_M:      // Waiting for '<' / '>'
        mspPort->c_state = MSP_HEADER_V1;
        switch (c) {
        case '<':
            mspPort->packetType = MSP_PACKET_COMMAND;
            break;
        case '>':
            mspPort->packetType = MSP_PACKET_REPLY;
            break;
        default:
            mspPort->c_state = MSP_IDLE;
            break;
        }
        break;
    case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum ^= c;

        if (mspPort->offset == 2) {
            // Check incoming buffer size limit
            if (mspPort->inBuf[0] > MSP_PORT_INBUF_SIZE) {
                mspPort->c_state = MSP_IDLE;
            } else {
                mspPort->dataSize = mspPort->inBuf[0];
                mspPort->cmdMSP = mspPort->inBuf[1];
                mspPort->cmdFlags = 0;
                mspPort->offset = 0;                // re-use buffer
                mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }
        }
        break;

    case MSP_PAYLOAD_V1:
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum ^= c;
        if (mspPort->offset == mspPort->dataSize) {
            mspPort->c_state = MSP_CHECKSUM_V1;
        }
        break;

    case MSP_CHECKSUM_V1:
        if (mspPort->checksum == c) {
            mspPort->c_state = MSP_COMMAND_RECEIVED;
        } else {
            mspPort->c_state = MSP_IDLE;
        }
        break;
    }
    return true;
}

/**
 * calculate crc of data
 * @param checksum
 * @param data
 * @param len
 * @return
 */
static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len) {
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}

/**
 * encode packet in MSP Frame
 * @param mspPort
 * @param packet
 * @param mspVersion
 * @return
 */
static int mspSerialSend(mspPort_t *mspPort, mspPacket_t *packet) {
    const uint8_t dataLen = sbufBytesRemaining(&packet->buf);
    const uint8_t dir = (packet->direction ==MSP_DIRECTION_REQUEST) ? '<' : '>';
    const uint8_t status = (packet->result == MSP_RESULT_ERROR) ? '!' : dir;
    uint8_t checksum = 0;
    checksum ^= dataLen;
    checksum ^= packet->cmd;
    checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);

    // total = header + data + crc
    const int totalFrameLength = 6 + dataLen;

    //is enough space in buffer
    if (!isSerialTransmitBufferEmpty(mspPort->port) && ((int) serialTxBytesFree(mspPort->port) < totalFrameLength)) {
        return 0;
    }

    // Transmit frame
    serialBeginWrite(mspPort->port);
    serialWrite(mspPort->port, '$');
    serialWrite(mspPort->port, 'M');
    serialWrite(mspPort->port, status);
    serialWrite(mspPort->port, dataLen);
    serialWrite(mspPort->port, packet->cmd);
    serialWriteBuf(mspPort->port, sbufPtr(&packet->buf), dataLen);
    serialWrite(mspPort->port, checksum);
    serialEndWrite(mspPort->port);

    return totalFrameLength;
}

/**
 * push a new command to the MSP interface
 * @param mspPort
 * @param cmd
 * @param data
 * @param datalen
 * @param direction
 * @return
 */
int mspSerialPush(mspPort_t *mspPort, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction) {
    int ret = 0;
    mspPacket_t push = { .buf = { .ptr = data, .end = data + datalen, }, .cmd = cmd, .result = 0, .direction = direction, };

    ret = mspSerialSend(mspPort, &push);

    return ret; // return the number of bytes written
}

void mspDebugData(const uint8_t* data, uint16_t len) {
    mspSerialPush(debugPort, MSP_DEBUGMSG, (uint8_t*) data, len, MSP_DIRECTION_REPLY);
}

/**
 * command was received send reply
 * @param msp
 */
static void mspSerialProcessReceivedCommand(mspPort_t *msp) {

    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];
    mspPacket_t reply = { .buf = { .ptr = outBuf, .end = &outBuf[MSP_PORT_OUTBUF_SIZE - 1], }, .cmd = -1, .flags = 0, .result = 0, .direction =
            MSP_DIRECTION_REPLY, };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = { .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, }, .cmd = msp->cmdMSP, .flags = msp->cmdFlags, .result = 0,
            .direction = MSP_DIRECTION_REQUEST, };

    const mspResult_e status = msp->mspProcessCommandFnPtr(&command, &reply);
    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialSend(msp, &reply);
    }
}

static void mspSerialProcessReceivedReply(mspPort_t *msp) {
    mspPacket_t command = { .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, }, .cmd = msp->cmdMSP, .flags = msp->cmdFlags, .result = 0,
            .direction = MSP_DIRECTION_REQUEST, };
    msp->mspProcessReplyFnPtr(&command);

}
/**
 * read all data which arrived at mspPort and take action if message was complete
 * @param mspPort
 */
void mspProcess(mspPort_t *mspPort) {
    if (serialRxBytesWaiting(mspPort->port)) {
        mspPort->lastActivityMs = 0; // Todo

        while (serialRxBytesWaiting(mspPort->port)) {
            const uint8_t c = serialRead(mspPort->port);
            mspSerialProcessReceivedData(mspPort, c);
            if (mspPort->c_state == MSP_COMMAND_RECEIVED) {

                if (mspPort->packetType == MSP_PACKET_COMMAND) {
                    mspSerialProcessReceivedCommand(mspPort);
                } else if (mspPort->packetType == MSP_PACKET_REPLY) {
                    mspSerialProcessReceivedReply(mspPort);
                }
                mspPort->c_state = MSP_IDLE;
                break; // process one command at a time so as not to block.
            }
        }
    }
}

