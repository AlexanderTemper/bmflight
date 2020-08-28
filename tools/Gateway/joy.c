#include "joy.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>

static int joyDev;
/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
static int read_event(int fd, struct js_event *event) {
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

rx_joy_t rx_joy;

int initJoy(const char* name) {
    rx_joy.arm = 1000;
    rx_joy.roll = 1500;
    rx_joy.pitch = 1500;
    rx_joy.yaw = 1500;
    rx_joy.throttle = 1000;
    joyDev = open(name, O_RDONLY);
    return joyDev;
}

void readJoy(void) {
    struct js_event event;
    read_event(joyDev, &event);
    switch (event.type) {
    case JS_EVENT_BUTTON:
        if (event.number == 5) {
            rx_joy.arm = event.value * 2000;
        }

        //printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
        break;
    case JS_EVENT_AXIS:
        switch (event.number) {
        case 0:
            rx_joy.yaw = (((float)event.value/32767)*500)+1500;
            break;
        case 1:
            rx_joy.throttle = ((float)-event.value/32767)*1000+1000;
            break;
        case 3:
            rx_joy.roll = (((float)event.value/32767)*500)+1500;
            break;
        case 4:
            rx_joy.pitch = (((float)-event.value/32767)*500)+1500;
            break;
        default:
            /* Ignore init events. */
            break;
        }
        //axis = get_axis_state(&event, axes);
//        if (axis < 3)
//            printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
        //printf("axis %u %6d\n", event.number, event.value);
        break;
    default:
        /* Ignore init events. */
        break;
    }
    printf("roll %6d, pitch %6d, yaw %6d, thrust %6d, arm %d \n", rx_joy.roll, rx_joy.pitch, rx_joy.yaw, rx_joy.throttle, rx_joy.arm);
}
