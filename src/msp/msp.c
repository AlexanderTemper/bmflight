#include "drivers/serial.h"
#include "common/fc.h"
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
    const uint8_t status = (packet->result == MSP_RESULT_ERROR) ? '!' : '>';
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


    mspSerialPush(debugPort, MSP_DEBUGMSG, (uint8_t*)data, len, MSP_DIRECTION_REPLY);
}

static bool mspProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst) {
    switch (cmdMSP) {
    case MSP_BATTERY_CONFIG: //TODO
        sbufWriteU8(dst, (330 + 5) / 10);
        sbufWriteU8(dst, (430 + 5) / 10);
        sbufWriteU8(dst, (350 + 5) / 10);
        sbufWriteU16(dst, 2500);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 330);
        sbufWriteU16(dst, 430);
        sbufWriteU16(dst, 350);
        break;
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;
    case MSP_FC_VARIANT:
        sbufWriteString(dst, FC_VARIANT);
        break;
    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO: { //TODO check info
        sbufWriteData(dst, BOARD_IDENTIFIER, 4);
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection
        sbufWriteU8(dst, 0);  // 0 == FC
        sbufWriteU8(dst, 0); // Target capabilities
        sbufWriteU8(dst, 4); // Name length
        sbufWriteData(dst, "TEST", 4);
        sbufWriteU8(dst, 0); //USE_BOARD_INFO
        sbufWriteU8(dst, 0); //USE_BOARD_INFO
        uint8_t emptySignature[32];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
        sbufWriteU8(dst, 255);
        break;
    }

    case MSP_BUILD_INFO: {
        const char * const shortGitRevision = "NO INFO";
        const char * const buildDate = __DATE__;
        const char * const buildTime = __TIME__;
        sbufWriteData(dst, buildDate, 11);
        sbufWriteData(dst, buildTime, 8);
        sbufWriteData(dst, shortGitRevision, 7);
        break;
    }
    case MSP_ANALOG:
        sbufWriteU8(dst, 3);
        sbufWriteU16(dst, 0); // milliamp hours drawn from battery
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0); // send current in 0.01 A steps, range is -320A to 320A
        sbufWriteU16(dst, 0);
        break;
    case MSP_DEBUG: {
        int16_t debug[4]; //todo
        for (int i = 0; i < 4; i++) {
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        }
        break;
    }
    case MSP_DEBUGMSG:
        break;
    case MSP_UID:
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 1);
        sbufWriteU32(dst, 2);
        break;
    case MSP_FEATURE_CONFIG:
        sbufWriteU32(dst, 0); //Todo
        break;
    default:
        return false;
    }
    return true;
}

static mspResult_e mspProcessInCommand(uint8_t cmdMSP, sbuf_t *src) {
    //const unsigned int dataSize = sbufBytesRemaining(src);
    switch (cmdMSP) {
    case MSP_SET_RAW_RC:
        //todo
        break;
    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

/**
 * process the command
 * @param cmd
 * @param reply
 * @return
 */
static mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply) {
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspProcessOutCommand(cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    } else {
        ret = mspProcessInCommand(cmdMSP, src);
    }
    reply->result = ret;
    return ret;
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

    const mspResult_e status = mspFcProcessCommand(&command, &reply);
    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialSend(msp, &reply);
    }
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
                    // we ignore MSP_PACKET_REPLY for now because FC is MSP Slave
                }
                mspPort->c_state = MSP_IDLE;
                break; // process one command at a time so as not to block.
            }
        }
    }
}

