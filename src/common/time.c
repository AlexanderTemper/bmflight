#include "common/time.h"

static timeMs_t (*milliFuncPtr)(void);
static timeMs_t (*microsFuncPtr)(void);
static void (*delayNanoSecondsFuncPtr)(timeUs_t time);

/**
 * init Time functions has to be called before other function of this module
 * @param millis
 * @param micros
 */
void initTime(timeMs_t (*milliFnP)(void), timeUs_t (*microsFnP)(void), void (*delayNanoSecondsFnP)(uint32_t nsec)) {
    milliFuncPtr = milliFnP;
    microsFuncPtr = microsFnP;
    delayNanoSecondsFuncPtr = delayNanoSecondsFnP;
}
/**
 * returns the time in ms since start
 * @return
 */
timeMs_t millis(void) {
    return milliFuncPtr();
}

/**
 * returns the time in us since start
 * @return
 */
timeUs_t micros(void) {
    return microsFuncPtr();
}

/**
 * delay the the execution
 * @param nsec to delay the execution
 */
void delayNanoSeconds(uint32_t nsec) {
    delayNanoSecondsFuncPtr(nsec);
}
