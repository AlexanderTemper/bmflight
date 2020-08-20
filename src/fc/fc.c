#include "fc/fc.h"
#include "imu/imu.h"
#include "imu/betaflightIMU.h"

attitude_estimator_t attitude_estimator;
void initFC(void){
    attitude_estimator.init = &betaIMUInit;
    attitude_estimator.update = &updatebetaIMU;
    initImu(&attitude_estimator);
}

