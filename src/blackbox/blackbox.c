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

#ifdef USE_BLACKBOX

#include "blackbox/blackbox.h"
#include "msp/msp.h"
#include "common/streambuf.h"

#define DEFAULT_BLACKBOX_DEVICE  BLACKBOX_DEVICE_NONE
// number of flight loop iterations before logging I-frame
static int16_t blackboxIInterval = 0;
//number of flight loop iterations before logging P-frame
static int16_t blackboxPInterval = 0;
static int32_t blackboxSInterval = 0;

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
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_PREDICTOR,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_ENCODING,
    BLACKBOX_STATE_SEND_SLOW_HEADER,
    BLACKBOX_STATE_SEND_SYSINFO,
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
int dummy = -1000;
static uint32_t loopIteration = 0;
static void writeSysInfo(void) {
    sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES]);
    blackboxWriteUnsignedVB('I'); //Todo p frames

    blackboxWriteUnsignedVB(loopIteration++);
    blackboxWriteUnsignedVB(micros());
    blackboxWriteSignedVB(dummy);
    blackboxWriteSignedVB(0);
    blackboxWriteSignedVB(0);

    sbufSwitchToReader(&logBuffer, &logLineData[0]);
    mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));
    dummy++;
}
static const uint8_t blackboxHeader[] = "H Product:Blackbox flight data recorder by Nicholas Sherlock\nH Data version:2\n";
static const uint8_t headerName[] = "H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2]\n";
static const uint8_t headerPredictor[] = "H Field I predictor:0,0,0,0,0\n";
static const uint8_t headerEncoding[] = "H Field I encoding:1,1,0,0,0\n";

static const uint8_t pHeaderPredictor[] = "H Field P predictor:6,2,1,1,1\n";
static const uint8_t pHheaderEncoding[] = "H Field P encoding:9,0,0,0,0\n";
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
        break;
    case BLACKBOX_STATE_SEND_START:
        //if (ARMING_FLAG(ARMED)) {
        mspWriteBlackBoxData(blackboxHeader, sizeof(blackboxHeader));
        blackboxState = BLACKBOX_STATE_SEND_HEADER;
        //}
        break;

    case BLACKBOX_STATE_SEND_HEADER: {
        //I and P definition
        const uint8_t iInterval[] = "H I interval: 16\n"; //todo use blackboxIInterval
        mspWriteBlackBoxData(iInterval, sizeof(iInterval));
        const uint8_t pInterval[] = "H P interval: 1/16\n"; //todo use blackboxPInterval (currently no P frame)
        mspWriteBlackBoxData(pInterval, sizeof(pInterval));
        blackboxState = BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_NAMES;
    }
        break;

    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_NAMES:
        mspWriteBlackBoxData(headerName, sizeof(headerName));
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
        blackboxState = BLACKBOX_STATE_SEND_SYSINFO;
        break;

    case BLACKBOX_STATE_SEND_SYSINFO:
        writeSysInfo();
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
    if (enabled) {
        blackboxConfig_System.device = BLACKBOX_DEVICE_SERIAL;
    }
    //blackboxResetIterationTimers();

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
