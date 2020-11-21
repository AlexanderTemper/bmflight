#pragma once

#include <stdbool.h>
#include <stdint.h>

#define NULL ((void *)0)
typedef enum {
    X = 0,
    Y,
    Z,
    XYZ_AXIS_COUNT
} axis_e;

#define USE_BLACKBOX

#define TARGET_LOOP_HZ 500

#ifdef FC_TARGET_SITL
    #include <stdio.h>
#endif

