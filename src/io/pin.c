#include "io/pin.h"

static status_leds_t status_leds;
static void (*setLevelFuncPointer)(status_leds_e pinId, bool level);

void initStatusLed(void (*setLevelPtr)(status_leds_e pinId, bool level)) {
    setLevelFuncPointer = setLevelPtr;
}

void setStatusLedLevel(status_leds_e led, bool level) {
    status_leds.pin[led].level = level;
    setLevelFuncPointer(led, level);
}

bool getStatusLedLevel(status_leds_e led) {
    return status_leds.pin[led].level;
}
