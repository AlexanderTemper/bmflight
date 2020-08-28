#pragma once

typedef struct rx_joy_s {
    int roll;
    int pitch;
    int yaw;

    int throttle;
    int arm;
} rx_joy_t;


extern rx_joy_t rx_joy;
int initJoy(const char* name);
void readJoy(void);
