#pragma once

#include "global.h"

// time difference
typedef int32_t timeDelta_t;
// millisecond time (wrap around 50days)
typedef uint32_t timeMs_t;
// microseconds time (wrap around 71 minutes)
typedef uint32_t timeUs_t;

/**
 * calculate time difference between a-b
 * @param a
 * @param b
 * @return
 */
static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) {
    return (timeDelta_t) (a - b);
}

/**
 * init Time functions has to be called before other function of this module
 * @param millis
 * @param micros
 */
void initTime(timeMs_t (*milliFnP)(void), timeUs_t (*microsFnP)(void), void (*delayNanoSecondsFnP)(uint32_t nsec));

/**
 * returns the time in ms since start
 * @return
 */
timeMs_t millis(void);

/**
 * returns the time in us since start
 * @return
 */
timeUs_t micros(void);

/**
 * delay the the execution
 * @param nsec to delay the execution
 */
void delayNanoSeconds(uint32_t nsec);
