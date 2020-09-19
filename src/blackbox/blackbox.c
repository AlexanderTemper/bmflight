/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "global.h"
#include "stdarg.h"
#include "stdio.h"
#ifdef USE_BLACKBOX

#include "blackbox/blackbox.h"
#include "msp/msp.h"
#include "fc/fc.h"
#include "sensor/sensor.h"
#include "common/streambuf.h"
#include "common/sprintf.h"
#include "common/maths.h"

#define DEFAULT_BLACKBOX_DEVICE  BLACKBOX_DEVICE_NONE
// number of flight loop iterations before logging I-frame
static int16_t blackboxIInterval = 0;
//number of flight loop iterations before logging P-frame
static int16_t blackboxPInterval = 0;
static int32_t blackboxSInterval = 0;

static uint32_t blackboxIteration;
static uint16_t blackboxLoopIndex;
static uint16_t blackboxPFrameIndex;
static uint16_t blackboxIFrameIndex;

blackboxConfig_t blackboxConfig_System = {
    .p_ratio = 16,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .record_acc = 1,
    .mode = BLACKBOX_MODE_NORMAL };

blackboxConfig_t* blackboxConfig(void) {
    return &blackboxConfig_System;
}

uint8_t blackboxGetRateDenom(void) {
    return blackboxPInterval;
}

typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_SEND_START,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_NAMES,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_SIGNED,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_PREDICTOR,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_ENCODING,
    BLACKBOX_STATE_SEND_EXTRA_HEADERS,
    BLACKBOX_STATE_SEND_LOG,
    BLACKBOX_STATE_SHUTTING_DOWN,
} BlackboxState;

static BlackboxState blackboxState;

#define LOG_LINE_MAX_BYTES 128
//log buffer to wirte data
static uint8_t logLineData[LOG_LINE_MAX_BYTES];
static sbuf_t logBuffer;

static void blackboxWrite(uint8_t value) {
    sbufWriteU8(&logBuffer, value);
}
/**
 * Write an unsigned integer to the blackbox using variable byte encoding.
 */
static void blackboxWriteUnsignedVB(uint32_t value) {
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        blackboxWrite((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    blackboxWrite(value);
}
/**
 * ZigZag encoding maps all values of a signed integer into those of an unsigned integer in such
 * a way that numbers of small absolute value correspond to small integers in the result.
 *
 * (Compared to just casting a signed to an unsigned which creates huge resulting numbers for
 * small negative integers).
 */
static uint32_t zigzagEncode(int32_t value) {
    return (uint32_t) ((value << 1) ^ (value >> 31));
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
static void blackboxWriteSignedVB(int32_t value) {
    //ZigZag encode to make the value always positive
    blackboxWriteUnsignedVB(zigzagEncode(value));
}

static void blackboxWriteString(const char *s) {
    const uint8_t *pos = (uint8_t*) s;
    while (*pos) {
        blackboxWrite(*pos);
        pos++;
    }
}

static void _putc(void *p, char c) {
    (void) p;
    blackboxWrite(c);
}

static void blackboxPrintfv(const char *fmt, va_list va) {
    tfp_format(NULL, _putc, fmt, va);
}

/**
 * Cast the in-memory representation of the given float directly to an int.
 *
 * This is useful for printing the hex representation of a float number (which is considerably cheaper
 * than a full decimal float formatter, in both code size and output length).
 */
static uint32_t castFloatBytesToInt(float f) {
    union floatConvert_t {
        float f;
        uint32_t u;
    } floatConvert;

    floatConvert.f = f;

    return floatConvert.u;
}
/*
 * printf a Blackbox header line with a leading "H " and trailing "\n" added automatically.
 */
static void blackboxPrintfHeaderLine(const char *name, const char *fmt, ...) {
    va_list va;

    blackboxWrite('H');
    blackboxWrite(' ');
    blackboxWriteString(name);
    blackboxWrite(':');

    va_start(va, fmt);

    blackboxPrintfv(fmt, va);

    va_end(va);

    blackboxWrite('\n');
}

static void blackboxResetIterationTimers(void) {
    blackboxIteration = 0;
    blackboxLoopIndex = 0;
    blackboxIFrameIndex = 0;
    blackboxPFrameIndex = 0;
}
// Called once every FC loop in order to keep track of how many FC loop iterations have passed
static void blackboxAdvanceIterationTimers(void) {
    ++blackboxIteration;

    if (++blackboxLoopIndex >= blackboxIInterval) {
        blackboxLoopIndex = 0;
        blackboxIFrameIndex++;
        blackboxPFrameIndex = 0;
    } else if (++blackboxPFrameIndex >= blackboxPInterval) {
        blackboxPFrameIndex = 0;
    }
}
static bool blackboxShouldLogPFrame(void) {
    return blackboxPFrameIndex == 0 && blackboxConfig()->p_ratio != 0;
}

static bool blackboxShouldLogIFrame(void) {
    return blackboxLoopIndex == 0;
}

//static const uint8_t headerName[] = "H Field I name:loopIteration,time,"
//        "axisP[0],axisP[1],axisP[2],"
//        "axisI[0],axisI[1],axisI[2],"
//        "axisD[0],axisD[1],axisD[2],"
//        "rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3]\n";
static void writeIFrame(void) {

    sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES]);
    blackboxWriteUnsignedVB('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(micros());

    pid_debug_t* pidDebug = &getFcDebug()->pid_debug;
    // axisP
    blackboxWriteSignedVB(pidDebug->p[X]);
    blackboxWriteSignedVB(pidDebug->p[Y]);
    blackboxWriteSignedVB(pidDebug->p[Z]);
    // axisI
    blackboxWriteSignedVB(pidDebug->i[X]);
    blackboxWriteSignedVB(pidDebug->i[Y]);
    blackboxWriteSignedVB(pidDebug->i[Z]);
    // axisD
    blackboxWriteSignedVB(pidDebug->d[X]);
    blackboxWriteSignedVB(pidDebug->d[Y]);
    blackboxWriteSignedVB(pidDebug->d[Z]);

    // gyroADC
    gyroDev_t* gyro = &getSonsors()->gyro;
    blackboxWriteSignedVB(gyro->raw[X]);
    blackboxWriteSignedVB(gyro->raw[Y]);
    blackboxWriteSignedVB(gyro->raw[Z]);

    // rcCommand
    command_t * fcCommand = &getFcControl()->fc_command;
    blackboxWriteSignedVB(fcCommand->roll);
    blackboxWriteSignedVB(fcCommand->pitch);
    blackboxWriteSignedVB(fcCommand->yaw);
    blackboxWriteUnsignedVB(fcCommand->throttle);

    // motor
    motors_command_t * motorCommand = &getFcControl()->motor_command;
    blackboxWriteUnsignedVB(motorCommand->value[0] - getFcConfig()->MINTHROTTLE);
    blackboxWriteSignedVB(motorCommand->value[1] - motorCommand->value[0]);
    blackboxWriteSignedVB(motorCommand->value[2] - motorCommand->value[0]);
    blackboxWriteSignedVB(motorCommand->value[3] - motorCommand->value[0]);

    sbufSwitchToReader(&logBuffer, &logLineData[0]);
    printf("\n------------logIframe %d l = %d   ", blackboxIteration, sbufBytesRemaining(&logBuffer));
    mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));
}

static void writePFrame(void) {
    sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES]);
    blackboxWriteUnsignedVB('P');

    sbufSwitchToReader(&logBuffer, &logLineData[0]);
    printf("logPframe %d l = %d   ", blackboxIteration, sbufBytesRemaining(&logBuffer));
    mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));
}

// Called once every FC loop in order to log the current state
static void blackboxLogIteration(timeUs_t currentTimeUs) {
    if (blackboxShouldLogIFrame()) {
        writeIFrame();
    } else {
        if (blackboxShouldLogPFrame()) {
            //writePFrame();
        }
    }
}

static const uint8_t blackboxHeader[] = "H Product:Blackbox flight data recorder by Nicholas Sherlock\nH Data version:2\n";
static const uint8_t headerName[] = "H Field I name:loopIteration,time"
        ",axisP[0],axisP[1],axisP[2]"
        ",axisI[0],axisI[1],axisI[2]"
        ",axisD[0],axisD[1],axisD[2]"
        ",gyroADC[0],gyroADC[1],gyroADC[2]"
        ",rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3]"
        ",motor[0],motor[1],motor[2],motor[3]"
        "\n";
//1 = signed
//0 = unsigned
static const uint8_t headerSigned[] = "H Field I signed:0,0"
        ",1,1,1" //p
        ",1,1,1"//i
        ",1,1,1"//d
        ",1,1,1"//gyroADC
        ",1,1,1,0"//rcCommand
        ",0,0,0,0"//motor
        "\n";
//0 = Predict zero
//1 = Predict last value
//5 = Predict motor[0]
//11 = MINMOTOR
static const uint8_t headerPredictor[] = "H Field I predictor:0,0"
        ",0,0,0" //p
        ",0,0,0"//i
        ",0,0,0"//d
        ",0,0,0"//gyroADC
        ",0,0,0,0"//rcCommand
        ",11,5,5,5"//motor
        "\n";

//0 = signed
//1 = unsigned
static const uint8_t headerEncoding[] = "H Field I encoding:1,1"
        ",0,0,0" //p
        ",0,0,0"//i
        ",0,0,0"//d
        ",0,0,0"//gyroADC
        ",0,0,0,1"//rcCommand
        ",1,0,0,0"//motor
        "\n";

//0 = Predict zero
//1 = Predict last value
//3 = Predict average 2
static const uint8_t pHeaderPredictor[] = "H Field P predictor:6,2"
        ",1,1,1" //p
        ",1,1,1"//i
        ",1,1,1"//d
        ",3,3,3"//gyroADC
        ",1,1,1,1"//rcCommand
        ",3,3,3,3"//motor
        "\n";
//0 = signed
//1 = unsigned
//7 = TAG2_3S32
//9 = NULL
static const uint8_t pHheaderEncoding[] = "H Field P encoding:9,0"
        ",0,0,0" //p
        ",0,0,0"//i
        ",0,0,0"//d
        ",0,0,0"//gyroADC
        ",0,0,0,1"//rcCommand
        ",1,1,1,1"//motor
        "\n";

void blackboxStart(void) {
    if (blackboxState == BLACKBOX_STATE_STOPPED) {
        blackboxState = BLACKBOX_STATE_SEND_START;
    }
}
void blackboxStop(void) {
    if (blackboxState != BLACKBOX_STATE_STOPPED) {
        blackboxState = BLACKBOX_STATE_SHUTTING_DOWN;
    }
}

void blackboxUpdate(timeUs_t currentTimeUs) {
    switch (blackboxState) {
    case BLACKBOX_STATE_STOPPED:
        if (getFcStatus()->ARMED) {
            blackboxState = BLACKBOX_STATE_SEND_START;
        }
        break;
    case BLACKBOX_STATE_SEND_START:
        //if (ARMING_FLAG(ARMED)) {
        mspWriteBlackBoxData(blackboxHeader, sizeof(blackboxHeader));
        blackboxState = BLACKBOX_STATE_SEND_HEADER;
        //}
        break;

    case BLACKBOX_STATE_SEND_HEADER: {
        //reset log Buffer
        sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES]);
        //I and P definition
        blackboxPrintfHeaderLine("I interval", "%d", blackboxIInterval);
        blackboxPrintfHeaderLine("P interval", "%s", "1/16");
        sbufSwitchToReader(&logBuffer, &logLineData[0]);
        mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));
        blackboxState = BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_NAMES;
    }
        break;

    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_NAMES:
        mspWriteBlackBoxData(headerName, sizeof(headerName));
        blackboxState = BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_SIGNED;
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_SIGNED:
        mspWriteBlackBoxData(headerSigned, sizeof(headerSigned));
        blackboxState = BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_PREDICTOR;
        break;

    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_PREDICTOR:
        mspWriteBlackBoxData(headerPredictor, sizeof(headerPredictor));
        mspWriteBlackBoxData(pHeaderPredictor, sizeof(pHeaderPredictor));
        blackboxState = BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_ENCODING;
        break;

    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_ENCODING:
        mspWriteBlackBoxData(headerEncoding, sizeof(headerEncoding));
        mspWriteBlackBoxData(pHheaderEncoding, sizeof(pHheaderEncoding));
        blackboxState = BLACKBOX_STATE_SEND_EXTRA_HEADERS;
        break;
    case BLACKBOX_STATE_SEND_EXTRA_HEADERS: {
        gyroDev_t* gyro = &getSonsors()->gyro;
        //reset log Buffer
        sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES]);
        blackboxPrintfHeaderLine("minthrottle", "%d", getFcConfig()->MINTHROTTLE);
        blackboxPrintfHeaderLine("maxthrottle", "%d", getFcConfig()->MAXTHROTTLE);
        blackboxPrintfHeaderLine("gyro_scale", "0x%x", castFloatBytesToInt(gyro->scale * (M_PIf / 180) * 0.000001));
        sbufSwitchToReader(&logBuffer, &logLineData[0]);
        mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));
        blackboxState = BLACKBOX_STATE_SEND_LOG;
        break;
    }
    case BLACKBOX_STATE_SEND_LOG:
        if (!getFcStatus()->ARMED) {
            blackboxState = BLACKBOX_STATE_SHUTTING_DOWN;
            return;
        }
        blackboxLogIteration(currentTimeUs);
        blackboxAdvanceIterationTimers();
        break;

    case BLACKBOX_STATE_SHUTTING_DOWN: {
        const uint8_t endMarker[] = {
            'E',
            0xFF,
            'E',
            'n',
            'd',
            ' ',
            'o',
            'f',
            ' ',
            'l',
            'o',
            'g',
            0 };
        mspWriteBlackBoxData(endMarker, sizeof(endMarker));
        blackboxState = BLACKBOX_STATE_STOPPED;
    }
        break;
    }

}
/**
 * Call during system startup to initialize the blackbox.
 */
void blackboxInit(bool enabled) {
    blackboxResetIterationTimers();
    if (enabled) {
        blackboxConfig_System.device = BLACKBOX_DEVICE_SERIAL;
    }
    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    blackboxIInterval = (uint16_t) ((32 * TARGET_LOOP_HZ) / 1000);

    if (blackboxConfig()->p_ratio == 0) {
        blackboxPInterval = 0; // blackboxPInterval not used when p_ratio is zero, so just set it to zero
    } else if (blackboxConfig()->p_ratio > blackboxIInterval && blackboxIInterval >= 32) {
        blackboxPInterval = 1;
    } else {
        blackboxPInterval = blackboxIInterval / blackboxConfig()->p_ratio;
    }
    if (blackboxConfig()->device) {
        blackboxState = BLACKBOX_STATE_STOPPED;
    } else {
        blackboxState = BLACKBOX_STATE_DISABLED;
    }
    blackboxSInterval = blackboxIInterval * 256; // S-frame is written every 256*32 = 8192ms, approx every 8 seconds
}
#endif
