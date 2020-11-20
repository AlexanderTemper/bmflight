#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/Imu.h>
#include <rotors_control/common.h>
#include <sensor_msgs/Joy.h>

#include "common.h"

int rx[6];  // interval [1000;2000] for THROTTLE and [-500;+500] for

mav_msgs::RollPitchYawrateThrust control_msg_;

void joyCallback(const sensor_msgs::JoyConstPtr& msg) {
    // PS3 Control

    if(msg->axes[1]>0){
        rx[THROTTLE] = (msg->axes[1] * 1000) + 1000; //left y
    } else {
        rx[THROTTLE] = 1000;
    }

    rx[ROLL] = (-msg->axes[3] * 500) + 1500;
    rx[PITCH] = (msg->axes[4] * 500) + 1500;
    rx[YAW] = (-msg->axes[0] * 500) + 1500;


    rx[AUX1] = msg->buttons[5] * 2000;

    if (rx[THROTTLE] < 1050) { // Reset Things
        //setCurrentHeading();
        //resetAltitudePid();
    }
    //printf("joy: roll:%d,PITCH:%d,YAW:%d,THROTTLE:%d,AUX1:%d,AUX2:%d\n", rx[ROLL], rx[PITCH], rx[YAW], rx[THROTTLE], rx[AUX1], rx[AUX2]);
}

static void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("imuCallback got first imu message.");

    // Angular velocities data
    float gyroX = imu_msg->angular_velocity.x;
    float gyroY = imu_msg->angular_velocity.y;
    float gyroZ = imu_msg->angular_velocity.z;

    // Linear acceleration data
    float accX = imu_msg->linear_acceleration.x;
    float accY = imu_msg->linear_acceleration.y;
    float accZ = imu_msg->linear_acceleration.z;

    //printf("IMU Data: gyro:%f %f %f\n", gyroX, gyroY, gyroZ);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gateway_node");
    ros::NodeHandle nh;
    ROS_INFO_ONCE("Started gateway_node");
    ros::Subscriber imu_sub;
    ros::Subscriber joy_sub_;
    ros::Publisher ctrl_pub_;

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);
    if (args.size() != 2) {
        ROS_ERROR("ARGS ERROR");
        return 1;
    }
    std::string imu_topic = "/" + args.at(1) + "/imu";
    std::cout << imu_topic << "\n";

    joy_sub_ = nh.subscribe("joy", 1, &joyCallback);
    imu_sub = nh.subscribe(imu_topic, 1, imuCallback);
    ctrl_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust> (mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);

    control_msg_.roll = 0;
    control_msg_.pitch = 0;
    control_msg_.yaw_rate = 0;
    control_msg_.thrust.x = 0;
    control_msg_.thrust.y = 0;
    control_msg_.thrust.z = 0;

    ros::Rate r(250);

    while (ros::ok()) {

        control_msg_.roll = rx[ROLL];
        control_msg_.pitch = rx[PITCH];
        control_msg_.yaw_rate = rx[YAW];
        control_msg_.thrust.z = rx[THROTTLE];
        control_msg_.thrust.x = rx[AUX1];
        control_msg_.thrust.y = rx[AUX2];
        ros::Time update_time = ros::Time::now();
        control_msg_.header.stamp = update_time;
        control_msg_.header.frame_id = "rotors_joy_frame";
        ctrl_pub_.publish(control_msg_);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
