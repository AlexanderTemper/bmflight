#pragma once

#include "global.h"

typedef struct pin_s {
    bool level;
} pin_t;

typedef enum {
    ARM_LED = 0,
    CALIBRATION_LED,
    ERROR_LED,
    LEDS_COUNT
} status_leds_e;

typedef struct status_leds_s {
    pin_t pin[LEDS_COUNT];
} status_leds_t;

void initStatusLed(void (*setLevelPtr)(status_leds_e pinId, bool level));

void setStatusLedLevel(status_leds_e led, bool level);
bool getStatusLedLevel(status_leds_e led);
