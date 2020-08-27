#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <rotors_control/common.h>
#include <math.h>
#include <pthread.h>

typedef struct {
    int fd;
    struct sockaddr_in si;
    struct sockaddr_in recv;
    int port;
    char* addr;
    bool isServer;
} udpLink_t;

#define UDP_SIM_PORT 8810
typedef struct {
    double timestamp;                   // in seconds
    double imu_angular_velocity_rpy[3];
    double imu_linear_acceleration_xyz[3];
    double imu_orientation_quat[4];     //w, x, y, z
    double velocity_xyz[3];             // m/s, earth frame
    double position_xyz[3];             // meters, NED from origin
} sim_packet;

typedef struct {
    double timestamp;                   // in seconds
    uint16_t motor_speed[4]; // [1000-2000]
} servo_packet;

static servo_packet pwmPkt;

static udpLink_t stateLink, pwmLink;

static pthread_t udpWorker;
static pthread_mutex_t udpLock;
static ros::Publisher actuators_pub;

int udpInit(udpLink_t* link, const char* addr, int port, bool isServer) {
    int one = 1;

    if ((link->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        return -2;
    }

    setsockopt(link->fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)); // can multi-bind
    fcntl(link->fd, F_SETFL, fcntl(link->fd, F_GETFL, 0) | O_NONBLOCK); // nonblock

    link->isServer = isServer;
    memset(&link->si, 0, sizeof(link->si));
    link->si.sin_family = AF_INET;
    link->si.sin_port = htons(port);
    link->port = port;

    if (addr == NULL) {
        link->si.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        link->si.sin_addr.s_addr = inet_addr(addr);
    }

    if (isServer) {
        if (bind(link->fd, (const struct sockaddr *) &link->si, sizeof(link->si)) == -1) {
            return -1;
        }
    }
    return 0;
}

int udpSend(udpLink_t* link, const void* data, size_t size) {
    return sendto(link->fd, data, size, 0, (struct sockaddr *) &link->si, sizeof(link->si));
}

int udpRecv(udpLink_t* link, void* data, size_t size, uint32_t timeout_ms) {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(link->fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    if (select(link->fd + 1, &fds, NULL, NULL, &tv) != 1) {
        return -1;
    }

    socklen_t len;
    int ret;
    ret = recvfrom(link->fd, data, size, 0, (struct sockaddr *) &link->recv, &len);
    return ret;
}
//typedef struct {
//    float roll;
//    float pitch;
//    float yaw;
//    double timestamp;
//} attitude_t;
//
//
//static attitude_t real;
//void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
//    ROS_INFO_ONCE("odometryCallback got first odometry message.");
//    rotors_control::EigenOdometry odometry;
//    rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);
//    Eigen::Vector3d euler_angles;
//    mav_msgs::getEulerAnglesFromQuaternion(odometry.orientation, &euler_angles);
//
//    real.roll = euler_angles.x() * (1800.0f / M_PI);
//    real.pitch = euler_angles.y() * (1800.0f / M_PI);
//    real.yaw = -euler_angles.z() * (1800.0f / M_PI);
//
//    if (real.yaw < 0) {
//        real.yaw += 3600;
//    }
//
//    real.timestamp = odometry_msg->header.stamp.toSec();
//    //printf("odometry Data: [%f,%f,%f]\n", real.roll, real.pitch, real.yaw);
//}

float scale_angular_velocities(int16_t motor) {

    //((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    /// max_rot_velocity for model = 838 but this is to big
    int max_rot_velocity = 838 - 150;

    float temp = (float) (motor - 1000) / 1000; // Scale to 0-1
    if(motor< 1050){
        return 0;
    }
    return temp * max_rot_velocity + 150; //todo make me better
}
/**
 * thread for handling udp packages
 * @param data
 */
static void* udpThread(void* data) {
    (void) (data);
    while (true) {
        int n = udpRecv(&pwmLink, &pwmPkt, sizeof(servo_packet), 100);
        if (n == sizeof(servo_packet)) {
            //printf("get motor data %d %d %d %d\n", pwmPkt.motor_speed[0], pwmPkt.motor_speed[1], pwmPkt.motor_speed[2], pwmPkt.motor_speed[3]);
            mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
            actuator_msg->angular_velocities.clear();
            actuator_msg->angular_velocities.push_back(scale_angular_velocities(pwmPkt.motor_speed[0]));
            actuator_msg->angular_velocities.push_back(scale_angular_velocities(pwmPkt.motor_speed[1]));
            actuator_msg->angular_velocities.push_back(scale_angular_velocities(pwmPkt.motor_speed[2]));
            actuator_msg->angular_velocities.push_back(scale_angular_velocities(pwmPkt.motor_speed[3]));

            ros::Time current_time = ros::Time::now();
            actuator_msg->header.stamp.sec = current_time.sec;
            actuator_msg->header.stamp.nsec = current_time.nsec;

            actuators_pub.publish(actuator_msg);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
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

    sim_packet simPkg;
    simPkg.timestamp = imu_msg->header.stamp.toSec();

    simPkg.imu_angular_velocity_rpy[0] = (imu_msg->angular_velocity.x * (180 / M_PI)) * 16.3835;
    simPkg.imu_angular_velocity_rpy[1] = (imu_msg->angular_velocity.y * (180 / M_PI)) * 16.3835;
    simPkg.imu_angular_velocity_rpy[2] = (imu_msg->angular_velocity.z * (180 / M_PI)) * 16.3835;

    simPkg.imu_linear_acceleration_xyz[0] = (imu_msg->linear_acceleration.x / 9.8) * 1024;
    simPkg.imu_linear_acceleration_xyz[1] = (imu_msg->linear_acceleration.y / 9.8) * 1024;
    simPkg.imu_linear_acceleration_xyz[2] = (imu_msg->linear_acceleration.z / 9.8) * 1024;

    //printf("IMU Data: gyro:%f %f %f acc:%f %f %f\n", gyroX, gyroY, gyroZ, simPkg.imu_linear_acceleration_xyz[0], simPkg.imu_linear_acceleration_xyz[1], simPkg.imu_linear_acceleration_xyz[2]);

    udpSend(&stateLink, &simPkg, sizeof(simPkg));

    //printf("IMU Data: gyro:%f %f %f acc:%f %f %f\n", gyroX, gyroY, gyroZ, accX, accY, accZ);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sitl_interface");
    ros::NodeHandle nh;
    ROS_INFO_ONCE("Started position controller");
    ros::Subscriber imu_sub;
    ros::Subscriber odometry_sub;

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);
    if (args.size() != 2) {
        ROS_ERROR("ARGS ERROR");
        return 1;
    }
    std::string imu_topic = "/" + args.at(1) + "/imu";
    std::string odometry_topic = "/" + args.at(1) + "/odometry_sensor1/odometry";
    std::string actuators_pub_topic = "/" + args.at(1) + "/command/motor_speed";
    std::cout << imu_topic << "\n";
//
    ROS_INFO_ONCE("start UDP client...");
    int ret = udpInit(&stateLink, "127.0.0.1", UDP_SIM_PORT, false);
    if (ret != 0) {
        ROS_ERROR("CANT START UDP CLIENT");
        return 1;
    }

    ret = udpInit(&pwmLink, NULL, UDP_SIM_PORT + 1, true);
    printf("start UDP server...%d\n", ret);
    if (pthread_mutex_init(&udpLock, NULL) != 0) {
        ROS_ERROR("CANT START UDP SERVER");
        return 1;
    }

    actuators_pub = nh.advertise < mav_msgs::Actuators > (actuators_pub_topic, 1); //done before udpWorker boot
    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        ROS_ERROR("CANT START UDP WORKER");
        return 1;
    }

    imu_sub = nh.subscribe(imu_topic, 1, imuCallback);

    //odometry_sub = nh.subscribe(odometry_topic, 1, odometryCallback);
//
//
//    ros::NodeHandle nh2;
//
//    rotors_control::PositionControllerNode position_controller_node;

    ros::spin();

    return 0;
}
