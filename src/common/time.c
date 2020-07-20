#include "common/time.h"

static timeMs_t (*milliFuncPtr)(void);
static timeMs_t (*microsFuncPtr)(void);
/**
 * init Time functions
 * @param millis
 * @param micros
 */
void initTime(timeMs_t (*milliFnP)(void), timeUs_t (*microsFnP)(void)) {
    milliFuncPtr = milliFnP;
    microsFuncPtr = microsFnP;
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
