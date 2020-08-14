#include "fc/fc.h"
#include "imu/imu.h"
#include "imu/mahonyAHRS.h"

attitude_estimator_t attitude_estimator;
void initFC(void){
    attitude_estimator.update = &updateMahony;
    initImu(&attitude_estimator);
}

