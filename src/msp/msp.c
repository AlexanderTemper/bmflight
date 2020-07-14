#include "drivers/serial.h"
#include "msp/msp.h"

#include <stdio.h> // TODO Weg

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

static void mspSerialProcessReceivedCommand(mspPort_t *msp) {
    printf("command Reveived %d", msp->cmdMSP);

    // static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

//    mspPacket_t reply = {
//        .buf = { .ptr = outBuf, .end = ARRAYEND(outBuf), },
//        .cmd = -1,
//        .flags = 0,
//        .result = 0,
//        .direction = MSP_DIRECTION_REPLY,
//    };
//    uint8_t *outBufHead = reply.buf.ptr;
//
//    mspPacket_t command = {
//        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
//        .cmd = msp->cmdMSP,
//        .flags = msp->cmdFlags,
//        .result = 0,
//        .direction = MSP_DIRECTION_REQUEST,
//    };

//    mspPostProcessFnPtr mspPostProcessFn = NULL;
//    const mspResult_e status = mspProcessCommandFn(msp->descriptor, &command, &reply, &mspPostProcessFn);
//
//    if (status != MSP_RESULT_NO_REPLY) {
//        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
//        mspSerialEncode(msp, &reply, msp->mspVersion);
//    }

}
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
                    // todo Implement
                }

                mspPort->c_state = MSP_IDLE;
                break; // process one command at a time so as not to block.
            }
        }

    }
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

    const int dataLen = sbufBytesRemaining(&packet->buf);
    const uint8_t status = (packet->result == MSP_RESULT_ERROR) ? '!' : '>';
    uint8_t headerBuffer[3] = { '$', 'M', status };
    uint8_t checksum = 0;
    checksum ^= status; // size and status are part of checksum
    checksum ^= dataLen;
    checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);

    // total = header + data + crc
    const int totalFrameLength = 6 + dataLen;

    //is enough space in buffer
    if (!isSerialTransmitBufferEmpty(mspPort->port) && ((int) serialTxBytesFree(mspPort->port) < totalFrameLength)) {
        return 0;
    }
    // Transmit frame
    serialBeginWrite(mspPort->port);
    serialWriteBuf(mspPort->port, headerBuffer, 3);
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
