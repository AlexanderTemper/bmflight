#pragma once

#include "global.h"

// time difference
typedef int32_t timeDelta_t;
// millisecond time (wrap around 50days)
typedef uint32_t timeMs_t;
// microseconds time (wrap around 71 minutes)
typedef uint32_t timeUs_t;

/**
 * init Time functions
 * @param millis
 * @param micros
 */
void initTime(timeMs_t (*millis)(void), timeUs_t (*micros)(void));
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
