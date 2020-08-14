#include "sensor/sensor.h"


static sensors_t *sensors;



void InitSonsors(sensors_t * s){
    sensors = s;
}


sensors_t * getSonsors(void){
    return sensors;
}
