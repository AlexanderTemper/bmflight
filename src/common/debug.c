#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "common/debug.h"

msp_debug_data_t msp_debug_data;

// local function pointer to debug interface
static sendDebugFuncPtr debugFuncPtr;

/**
 * initialize the debug Interface
 * this has to be called before any other function of Debug
 * @param ptr
 */
void initDebug(sendDebugFuncPtr ptr) {
    debugFuncPtr = ptr;
}

/**
 * write len data to debug interface
 * @param data
 * @param len
 */
void debugData(const uint8_t *data, uint16_t len) {
    debugFuncPtr(data, len);
}

/**
 * write an int16_t to the debug interface
 * @param value
 */
void printInt16Debug(int16_t value) {
    char buffer[12];
    sprintf(buffer, "%"PRId16, value);
    printDebug(buffer);
}

/**
 * write an int32_t to the debug interface
 * @param value
 */
void printInt32Debug(int32_t value) {
    char buffer[12];
    sprintf(buffer, "%"PRId32, value);
    printDebug(buffer);
}

/**
 * debug an string
 * @param string
 */
void printDebug(const char *string) {
    uint16_t len = strlen(string);
    if (len > 256) {
        return;
    }
    debugFuncPtr((uint8_t*) string, len);
}
