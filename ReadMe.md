---------------------------------------------------------

# Lightweight Flight Controller
A simple implementation of a flight controller for weaker MCUs.

The firmware was developed for the BMF055 of Bosch Sensortec. The BMF055 is a SoC which includes three sensors and an samd20j18a as MCU. The included sensors are an accelerometer, gyroscope and an magnetometer therefore all it needs to be used as an flight controller for drones.

Problem is only the samd20j18a which lacks in computational power to drive modern firmwares like Betaflight, Cleanflight or ArduPilot.
Therefore we focus in this firmware on the basic operations for an drone and strip unnecessary features. 

The Firmware is partial compatible with the [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/) and [Betaflight Blackbox Explorer](https://github.com/betaflight/blackbox-log-viewer) and uses an HAL so porting can be done to other MCUs. For testing and research an SITL target was defined. 

This SITL target can then be connected to an Simulator(Gazebo/ROS). With the Gateway Tool an Joystick can be used to control the drone.

![](https://raw.githubusercontent.com/AlexanderTemper/bmflight/master/doc/img/sim2StandAlone.png)

