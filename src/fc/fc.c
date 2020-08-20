#include "fc/fc.h"
#include "imu/imu.h"
#include "config/feature.h"


void initFC(void){
    initImu();
    featureEnable(FEATURE_RX_MSP);
}

