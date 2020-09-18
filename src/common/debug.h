#pragma once

#include "global.h"

//#define MSP_DEBUG_GYRO_TRIM
#define MSP_DEBUG_ACC_DATA
//#define MSP_DEBUG_GYRO_AVG

typedef struct msp_debug_data_s {
    uint16_t data[4];
} msp_debug_data_t;

extern msp_debug_data_t msp_debug_data;

typedef void (*sendDebugFuncPtr)(const uint8_t* data, uint16_t len);

/**
 * initialize the debug Interface
 * this has to be called before any other function of Debug
 * @param ptr
 */
void initDebug(sendDebugFuncPtr ptr);

/**
 * write len data to debug interface
 * @param data
 * @param len
 */
void debugData(const uint8_t *data, uint16_t len);

/**
 * write an int16_t to the debug interface
 * @param value
 */
void printInt16Debug(int16_t value);

/**
 * write an int32_t to the debug interface
 * @param value
 */
void printInt32Debug(int32_t value);

/**
 * debug an string
 * @param string
 */
void printDebug(const char *string);
