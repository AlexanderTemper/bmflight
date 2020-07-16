#include "common/debug.h"

static sendDebugFuncPtr debugFuncPtr;
//static uint8_t debugBuffer[256];


void initDebug(sendDebugFuncPtr ptr) {
    debugFuncPtr = ptr;
}

void debugData(const uint8_t *data,uint16_t len){
    debugFuncPtr(data,len);
}



void printDebug(const char *string)
{
    uint16_t len = strlen(string);
    if(len > 256){
       return;
    }
    debugFuncPtr((uint8_t*)string,len);
}
