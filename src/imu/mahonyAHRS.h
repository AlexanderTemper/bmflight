#pragma once

#include "global.h"
#include "common/time.h"

typedef struct {
    float w, x, y, z;
} quaternion;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww, wx, wy, wz, xx, xy, xz, yy, yz, zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

void updateMahony(timeUs_t currentTime);
