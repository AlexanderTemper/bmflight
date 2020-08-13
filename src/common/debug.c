#include "common/debug.h"
#include <string.h>
#include <stdio.h>

static sendDebugFuncPtr debugFuncPtr;
//static uint8_t debugBuffer[256];

void initDebug(sendDebugFuncPtr ptr) {
    debugFuncPtr = ptr;
}

void debugData(const uint8_t *data, uint16_t len) {
    debugFuncPtr(data, len);
}

void printInt16Debug(int16_t value) {
    char buffer[12];
    sprintf(buffer, "%d", value);
    printDebug(buffer);
}

void printInt32Debug(int32_t value) {
    char buffer[12];
    sprintf(buffer, "%ld", value);
    printDebug(buffer);
}

void printDebug(const char *string) {
    uint16_t len = strlen(string);
    if (len > 256) {
        return;
    }
    debugFuncPtr((uint8_t*) string, len);
}
