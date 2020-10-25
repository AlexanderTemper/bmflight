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
#include <stddef.h>

#include "global.h"
#include "stdarg.h"
//#include "stdio.h"
#ifdef USE_BLACKBOX

#include "blackbox/blackbox.h"
#include "msp/msp.h"
#include "fc/fc.h"
#include "fc/tasks.h"
#include "sensor/sensor.h"
#include "common/streambuf.h"
#include "common/sprintf.h"
#include "common/maths.h"
#include "common/debug.h"

#define BLACKBOX_PID_ENABLE
#define BLACKBOX_RC_COMMAND_ENABLE
#define BLACKBOX_ACC_ENABLE
#define BLACKBOX_MOTOR_ENABLE

#define BLACKBOX_DEBUG_TIMING
#define BLACKBOX_DEBUG_TIMING_ALL

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

typedef struct blackboxMainState_s {
    uint32_t time;
#ifdef BLACKBOX_PID_ENABLE
    int32_t axisPID_P[XYZ_AXIS_COUNT];
    int32_t axisPID_I[XYZ_AXIS_COUNT];
    int32_t axisPID_D[XYZ_AXIS_COUNT];
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
    int16_t rcCommand[4];
#endif
    int16_t gyroADC[XYZ_AXIS_COUNT];
#ifdef BLACKBOX_ACC_ENABLE
    int16_t accADC[XYZ_AXIS_COUNT];
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
    int16_t motor[4];
#endif
#ifdef BLACKBOX_DEBUG_TIMING
    int32_t debugTiming[8];
#endif
} blackboxMainState_t;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxMainState_t* blackboxHistory[3];

#define LOG_LINE_MAX_BYTES 128
//log buffer to wirte data
static uint8_t logLineData[LOG_LINE_MAX_BYTES];
static sbuf_t logBuffer;

static const uint8_t blackboxHeader[] = "H Product:Blackbox flight data recorder by Nicholas Sherlock\nH Data version:2\n";
static const uint8_t headerNamePart1[] = "H Field I name:loopIteration,time"
#ifdef BLACKBOX_PID_ENABLE
        ",axisP[0],axisP[1],axisP[2]"
        ",axisI[0],axisI[1],axisI[2]"
        ",axisD[0],axisD[1],axisD[2]"
#endif
                ",gyroADC[0],gyroADC[1],gyroADC[2]";
static const uint8_t headerNamePart2[] =
#ifdef BLACKBOX_ACC_ENABLE
        ",accSmooth[0],accSmooth[1],accSmooth[2]"
#endif
#ifdef BLACKBOX_DEBUG_TIMING
        ",debug[0],debug[1],debug[2],debug[3]"
#endif
#ifdef BLACKBOX_DEBUG_TIMING_ALL
        ",debug[4],debug[5],debug[6],debug[7]"
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
        ",rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3]"
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
        ",motor[0],motor[1],motor[2],motor[3]"
#endif
                "\n";
//1 = signed
//0 = unsigned
static const uint8_t headerSigned[] = "H Field I signed:0,0"
#ifdef BLACKBOX_PID_ENABLE
        ",1,1,1" //p
        ",1,1,1"//i
        ",1,1,1"//d
#endif
                ",1,1,1" //gyroADC
#ifdef BLACKBOX_ACC_ENABLE
        ",1,1,1" //accSmooth
#endif
#ifdef BLACKBOX_DEBUG_TIMING
        ",1,1,1,1" //debug timing
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
        ",1,1,1,0" //rcCommand
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
        ",0,0,0,0" //motor
#endif
        "\n";
//0 = Predict zero
//1 = Predict last value
//5 = Predict motor[0]
//11 = MINMOTOR
static const uint8_t headerPredictor[] = "H Field I predictor:0,0"
#ifdef BLACKBOX_PID_ENABLE
        ",0,0,0" //p
        ",0,0,0"//i
        ",0,0,0"//d
#endif
                ",0,0,0" //gyroADC
#ifdef BLACKBOX_ACC_ENABLE
        ",0,0,0" //accSmooth
#endif
#ifdef BLACKBOX_DEBUG_TIMING
        ",0,0,0,0" //debug timing
#endif
#ifdef BLACKBOX_DEBUG_TIMING_ALL
        ",0,0,0,0" //debug timing
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
        ",0,0,0,0" //rcCommand
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
        ",11,5,5,5" //motor
#endif
        "\n";

//0 = signed
//1 = unsigned
static const uint8_t headerEncoding[] = "H Field I encoding:1,1"
#ifdef BLACKBOX_PID_ENABLE
        ",0,0,0" //p
        ",0,0,0"//i
        ",0,0,0"//d
#endif
                ",0,0,0" //gyroADC
#ifdef BLACKBOX_ACC_ENABLE
        ",0,0,0" //accSmooth
#endif
#ifdef BLACKBOX_DEBUG_TIMING
        ",0,0,0,0" //debug timing
#endif
#ifdef BLACKBOX_DEBUG_TIMING_ALL
        ",0,0,0,0" //debug timing
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
        ",0,0,0,1" //rcCommand
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
        ",1,0,0,0" //motor
#endif
        "\n";

//0 = Predict zero
//1 = Predict last value
//2 = Predict straight line
//3 = Predict average 2
//6 = Predict increment
static const uint8_t pHeaderPredictor[] = "H Field P predictor:6,2"
#ifdef BLACKBOX_PID_ENABLE
        ",1,1,1" //p
        ",1,1,1"//i
        ",1,1,1"//d
#endif
                ",3,3,3" //gyroADC
#ifdef BLACKBOX_ACC_ENABLE
        ",3,3,3" //accSmooth
#endif
#ifdef BLACKBOX_DEBUG_TIMING
        ",1,1,1,1" //debug
#endif
#ifdef BLACKBOX_DEBUG_TIMING_ALL
        ",1,1,1,1" //debug
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
        ",1,1,1,1" //rcCommand
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
        ",3,3,3,3" //motor
#endif
        "\n";
//0 = signed
//1 = unsigned
//7 = TAG2_3S32
//8 = TAG8_4S16
//9 = NULL
static const uint8_t pHheaderEncoding[] = "H Field P encoding:9,0"
#ifdef BLACKBOX_PID_ENABLE
        ",0,0,0" //p
        ",0,0,0"//i
        ",0,0,0"//d
#endif
                ",0,0,0" //gyroADC
#ifdef BLACKBOX_ACC_ENABLE
        ",0,0,0" //accSmooth
#endif
#ifdef BLACKBOX_DEBUG_TIMING
        ",0,0,0,0" //debug
#endif
#ifdef BLACKBOX_DEBUG_TIMING_ALL
        ",0,0,0,0" //debug
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
        ",8,8,8,8" //rcCommand
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
        ",1,1,1,1" //motor
#endif
        "\n";

static void blackboxWrite(uint8_t value) {
    sbufWriteU8(&logBuffer, value);
}

/// Some Helper Function todo Move them in own file

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
void blackboxWriteTag8_4S16(int32_t *values) {

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
        FIELD_ZERO = 0,
        FIELD_4BIT = 1,
        FIELD_8BIT = 2,
        FIELD_16BIT = 3
    };

    uint8_t selector = 0;
    //Encode in reverse order so the first field is in the low bits:
    for (int x = 3; x >= 0; x--) {
        selector <<= 2;

        if (values[x] == 0) {
            selector |= FIELD_ZERO;
        } else if (values[x] < 8 && values[x] >= -8) {
            selector |= FIELD_4BIT;
        } else if (values[x] < 128 && values[x] >= -128) {
            selector |= FIELD_8BIT;
        } else {
            selector |= FIELD_16BIT;
        }
    }

    blackboxWrite(selector);

    int nibbleIndex = 0;
    uint8_t buffer = 0;
    for (int x = 0; x < 4; x++, selector >>= 2) {
        switch (selector & 0x03) {
        case FIELD_ZERO:
            //No-op
            break;
        case FIELD_4BIT:
            if (nibbleIndex == 0) {
                //We fill high-bits first
                buffer = values[x] << 4;
                nibbleIndex = 1;
            } else {
                blackboxWrite(buffer | (values[x] & 0x0F));
                nibbleIndex = 0;
            }
            break;
        case FIELD_8BIT:
            if (nibbleIndex == 0) {
                blackboxWrite(values[x]);
            } else {
                //Write the high bits of the value first (mask to avoid sign extension)
                blackboxWrite(buffer | ((values[x] >> 4) & 0x0F));
                //Now put the leftover low bits into the top of the next buffer entry
                buffer = values[x] << 4;
            }
            break;
        case FIELD_16BIT:
            if (nibbleIndex == 0) {
                //Write high byte first
                blackboxWrite(values[x] >> 8);
                blackboxWrite(values[x]);
            } else {
                //First write the highest 4 bits
                blackboxWrite(buffer | ((values[x] >> 12) & 0x0F));
                // Then the middle 8
                blackboxWrite(values[x] >> 4);
                //Only the smallest 4 bits are still left to write
                buffer = values[x] << 4;
            }
            break;
        }
    }
    //Anything left over to write?
    if (nibbleIndex == 1) {
        blackboxWrite(buffer);
    }
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

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(timeUs_t currentTimeUs) {
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxCurrent->time = currentTimeUs;

    gyroDev_t* gyro = &getSonsors()->gyro;
    accDev_t* acc = &getSonsors()->acc;
    pid_debug_t* pidDebug = &getFcDebug()->pid_debug;
    command_t * fcCommand = &getFcControl()->fc_command;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
#ifdef BLACKBOX_PID_ENABLE
        blackboxCurrent->axisPID_P[i] = pidDebug->p[i];
        blackboxCurrent->axisPID_I[i] = pidDebug->i[i];
        blackboxCurrent->axisPID_D[i] = pidDebug->d[i];
#endif
        blackboxCurrent->gyroADC[i] = gyro->raw[i];
#ifdef BLACKBOX_ACC_ENABLE
        blackboxCurrent->accADC[i] = acc->ADCRaw[i];
#endif
    }
#ifdef BLACKBOX_RC_COMMAND_ENABLE
    blackboxCurrent->rcCommand[ROLL] = fcCommand->roll;
    blackboxCurrent->rcCommand[PITCH] = fcCommand->pitch;
    blackboxCurrent->rcCommand[YAW] = fcCommand->yaw;
    blackboxCurrent->rcCommand[THROTTLE] = fcCommand->throttle;
#endif

#ifdef BLACKBOX_DEBUG_TIMING
#ifdef USE_TASK_STATISTICS
    task_t * loopTask = getTask(TASK_LOOP);
    task_t * attTask = getTask(TASK_ATTITUDE);
    blackboxCurrent->debugTiming[0] = loopTask->taskLatestDeltaTimeUs;
    blackboxCurrent->debugTiming[1] = attTask->taskLatestDeltaTimeUs;
    blackboxCurrent->debugTiming[2] = loopTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    blackboxCurrent->debugTiming[3] = attTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
#endif
#endif

#ifdef BLACKBOX_DEBUG_TIMING_ALL
#ifdef USE_TASK_STATISTICS
    task_t * task1 = getTask(TASK_SERIAL);
    task_t * task2 = getTask(TASK_SYSTEM);
    blackboxCurrent->debugTiming[4] = task1->taskLatestDeltaTimeUs;
    blackboxCurrent->debugTiming[5] = task2->taskLatestDeltaTimeUs;
    blackboxCurrent->debugTiming[6] = task1->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    blackboxCurrent->debugTiming[7] = task2->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
#endif
#endif

#ifdef BLACKBOX_MOTOR_ENABLE
    // motor
    motors_command_t * motorCommand = &getFcControl()->motor_command;
    for (int i = 0; i < 4; i++) {
        blackboxCurrent->motor[i] = motorCommand->value[i];
    }
#endif
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count) {
    int16_t *curr = (int16_t*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
    int16_t *prev1 = (int16_t*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
    int16_t *prev2 = (int16_t*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);

    for (int i = 0; i < count; i++) {
        // Predictor is the average of the previous two history states
        int32_t predictor = (prev1[i] + prev2[i]) / 2;

        blackboxWriteSignedVB(curr[i] - predictor);
    }
}

static void writeIFrame(void) {
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES - 1]);
    blackboxWriteUnsignedVB('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

#ifdef BLACKBOX_PID_ENABLE
    // axisP
    blackboxWriteSignedVB(blackboxCurrent->axisPID_P[X]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_P[Y]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_P[Z]);
    // axisI
    blackboxWriteSignedVB(blackboxCurrent->axisPID_I[X]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_I[Y]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_I[Z]);

    // axisD
    blackboxWriteSignedVB(blackboxCurrent->axisPID_D[X]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_D[Y]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_D[Z]);
#endif
    // gyroADC
    blackboxWriteSignedVB(blackboxCurrent->gyroADC[X]);
    blackboxWriteSignedVB(blackboxCurrent->gyroADC[Y]);
    blackboxWriteSignedVB(blackboxCurrent->gyroADC[Z]);

#ifdef BLACKBOX_ACC_ENABLE
    // acc
    blackboxWriteSignedVB(blackboxCurrent->accADC[X]);
    blackboxWriteSignedVB(blackboxCurrent->accADC[Y]);
    blackboxWriteSignedVB(blackboxCurrent->accADC[Z]);
#endif
#ifdef BLACKBOX_DEBUG_TIMING
    // debug
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[0]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[1]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[2]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[3]);
#endif
#ifdef BLACKBOX_DEBUG_TIMING_ALL
    // debug
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[4]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[5]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[6]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[7]);
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
    // rcCommand
    blackboxWriteSignedVB(blackboxCurrent->rcCommand[ROLL]);
    blackboxWriteSignedVB(blackboxCurrent->rcCommand[PITCH]);
    blackboxWriteSignedVB(blackboxCurrent->rcCommand[YAW]);
    blackboxWriteUnsignedVB(blackboxCurrent->rcCommand[THROTTLE]);
#endif

#ifdef BLACKBOX_MOTOR_ENABLE
    // motor
    blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - getFcConfig()->MINTHROTTLE);
    blackboxWriteSignedVB(blackboxCurrent->motor[1] - blackboxCurrent->motor[0]);
    blackboxWriteSignedVB(blackboxCurrent->motor[2] - blackboxCurrent->motor[0]);
    blackboxWriteSignedVB(blackboxCurrent->motor[3] - blackboxCurrent->motor[0]);
#endif

    sbufSwitchToReader(&logBuffer, &logLineData[0]);
    //printf("\n------%d,%d:I %d ", blackboxCurrent->time, blackboxIteration, sbufBytesRemaining(&logBuffer));
    mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

}

static void writePFrame(void) {

    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxLast = blackboxHistory[1];
    sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES - 1]);
    blackboxWriteUnsignedVB('P');

    //blackboxIteration no need to write since its increment 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));
#ifdef BLACKBOX_PID_ENABLE
    //P
    blackboxWriteSignedVB(blackboxCurrent->axisPID_P[X] - blackboxLast->axisPID_P[X]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_P[Y] - blackboxLast->axisPID_P[Y]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_P[Z] - blackboxLast->axisPID_P[Z]);

    //I
    blackboxWriteSignedVB(blackboxCurrent->axisPID_I[X] - blackboxLast->axisPID_I[X]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_I[Y] - blackboxLast->axisPID_I[Y]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_I[Z] - blackboxLast->axisPID_I[Z]);

    //D
    blackboxWriteSignedVB(blackboxCurrent->axisPID_D[X] - blackboxLast->axisPID_D[X]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_D[Y] - blackboxLast->axisPID_D[Y]);
    blackboxWriteSignedVB(blackboxCurrent->axisPID_D[Z] - blackboxLast->axisPID_D[Z]);
#endif
    //Since gyros, accs and motors are noisy, base their predictions on the average of the history:
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC), XYZ_AXIS_COUNT);
#ifdef BLACKBOX_ACC_ENABLE
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADC), XYZ_AXIS_COUNT);
#endif
#ifdef BLACKBOX_DEBUG_TIMING
    //debug
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[0] - blackboxLast->debugTiming[0]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[1] - blackboxLast->debugTiming[1]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[2] - blackboxLast->debugTiming[2]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[3] - blackboxLast->debugTiming[3]);
#endif
#ifdef BLACKBOX_DEBUG_TIMING_ALL
    //debug
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[4] - blackboxLast->debugTiming[4]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[5] - blackboxLast->debugTiming[5]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[6] - blackboxLast->debugTiming[6]);
    blackboxWriteSignedVB(blackboxCurrent->debugTiming[7] - blackboxLast->debugTiming[7]);
#endif
#ifdef BLACKBOX_RC_COMMAND_ENABLE
    int32_t deltas[4];
    for (int x = 0; x < 4; x++) {
        deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];
    }
    blackboxWriteTag8_4S16(deltas);
#endif
#ifdef BLACKBOX_MOTOR_ENABLE
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor), 4);
#endif
    sbufSwitchToReader(&logBuffer, &logLineData[0]);
    //printf("\n%d,%d:P %d ", blackboxCurrent->time, blackboxIteration, sbufBytesRemaining(&logBuffer));
    mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));

    //Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

// Called once every FC loop in order to log the current state
static void blackboxLogIteration(timeUs_t currentTimeUs) {

    if (blackboxShouldLogIFrame()) {
        loadMainState(currentTimeUs);
        writeIFrame();
    } else if (blackboxShouldLogPFrame()) {
        loadMainState(currentTimeUs);
        writePFrame();
    }
}

void blackboxStart(void) {
    if (blackboxState != BLACKBOX_STATE_DISABLED && blackboxState == BLACKBOX_STATE_STOPPED) {
        blackboxState = BLACKBOX_STATE_SEND_START;
    }
}
void blackboxStop(void) {
    if (blackboxState != BLACKBOX_STATE_DISABLED && blackboxState != BLACKBOX_STATE_STOPPED) {
        blackboxState = BLACKBOX_STATE_SHUTTING_DOWN;
    }
}

void blackboxUpdate(timeUs_t currentTimeUs) {
    switch (blackboxState) {
    case BLACKBOX_STATE_STOPPED:
        if (getFcStatus()->ARMED) {
            blackboxState = BLACKBOX_STATE_SEND_START;
            blackboxResetIterationTimers();
            blackboxHistory[0] = &blackboxHistoryRing[0];
            blackboxHistory[1] = &blackboxHistoryRing[1];
            blackboxHistory[2] = &blackboxHistoryRing[2];
            //No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it
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
        sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES - 1]);
        //I and P definition
        blackboxPrintfHeaderLine("I interval", "%d", blackboxIInterval);
        //blackboxPrintfHeaderLine("P interval", "%s", "1/16");
        blackboxPrintfHeaderLine("P interval", "%d", blackboxPInterval);
        //blackboxPrintfHeaderLine("P ratio", "%d", blackboxConfig()->p_ratio);
        sbufSwitchToReader(&logBuffer, &logLineData[0]);
        mspWriteBlackBoxData(logLineData, sbufBytesRemaining(&logBuffer));
        blackboxState = BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_NAMES;
    }
        break;

    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER_NAMES:
        mspWriteBlackBoxData(headerNamePart1, sizeof(headerNamePart1) - 1); //dont send enf of string here
        mspWriteBlackBoxData(headerNamePart2, sizeof(headerNamePart2));
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
        accDev_t* acc = &getSonsors()->acc;
        //reset log Buffer
        sbufInit(&logBuffer, &logLineData[0], &logLineData[LOG_LINE_MAX_BYTES - 1]);
        blackboxPrintfHeaderLine("minthrottle", "%d", getFcConfig()->MINTHROTTLE);
        blackboxPrintfHeaderLine("maxthrottle", "%d", getFcConfig()->MAXTHROTTLE);
        blackboxPrintfHeaderLine("gyro_scale", "0x%x", castFloatBytesToInt(gyro->scale * (M_PIf / 180) * 0.000001));
        blackboxPrintfHeaderLine("acc_1G", "%u", (uint16_t) ((float) 1 / acc->scale));
        blackboxPrintfHeaderLine("looptime", "%u", (uint16_t) (1000000 / TARGET_LOOP_HZ));

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
