#pragma once

#include "global.h"

typedef void (*sendDebugFuncPtr)(const uint8_t* data, uint16_t len);

void initDebug(sendDebugFuncPtr ptr);
void debugData(const uint8_t* data,uint16_t len);
void printDebug(const char *string);

void printInt16Debug(int16_t value);
void printInt32Debug(int32_t value);
