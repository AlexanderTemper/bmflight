#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

ros::Time imu_msg_head_stamp;
static void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("PositionController got first imu message.");

    // Angular velocities data
    float gyroX = imu_msg->angular_velocity.x;
    float gyroY = imu_msg->angular_velocity.y;
    float gyroZ = imu_msg->angular_velocity.z;

    // Linear acceleration data
    float accX = imu_msg->linear_acceleration.x;
    float accY = imu_msg->linear_acceleration.y;
    float accZ = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp = imu_msg->header.stamp;

   printf("IMU Data: gyro:%f %f %f acc:%f %f %f\n",gyroX,gyroY,gyroZ,accX,accY,accZ);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sitl_interface");
    ros::NodeHandle nh;
    ROS_INFO_ONCE("Started position controller");
    ros::Subscriber imu_sub;

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);
    if (args.size() != 2) {
        ROS_ERROR("ARGS ERROR");
        return 1;
    }
    std::string imu_topic = "/" + args.at(1) + "/imu";

    std::cout << imu_topic << "\n";
//
    imu_sub = nh.subscribe(imu_topic, 1, imuCallback);
//
//
//    ros::NodeHandle nh2;
//
//    rotors_control::PositionControllerNode position_controller_node;

    ros::spin();

    return 0;
}
