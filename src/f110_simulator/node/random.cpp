#define _GNU_SOURCE

#include <iostream>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// Action mapping: each row = {speed, steering_angle}
// 0: Forward
// 1: Forward + Left
// 2: Forward + Right
// 3: Stop
float mapping[4][2] = {
    {1.0, 0.0},   // forward
    {1.0, -1.0},  // forward + left
    {1.0, 1.0},   // forward + right
    {0.0, 0.0}    // stop
};

// Speed and angle scaling factors
float speed_limit = 1.8;
float angle_limit = 0.3;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Random_Controller");
    ros::NodeHandle n;

    // Publisher to send commands
    ros::Publisher command_pub =
        n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

    ros::Rate rate(0.5);  // publish every 2 seconds

    // Seed RNG once at startup
    std::srand(std::time(nullptr));

    while (ros::ok()) {
        // Pick a random action index (0–3)
        int index = std::rand() % 4;
        std::cout << "Random action index: " << index << std::endl;

        float speed = mapping[index][0];
        float angle = mapping[index][1];

        // Create AckermannDrive message
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.speed = speed * speed_limit;
        drive_msg.steering_angle = angle * angle_limit;

        // Wrap into a stamped message
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        drive_st_msg.header.stamp = ros::Time::now();
        drive_st_msg.drive = drive_msg;

        // Publish command
        command_pub.publish(drive_st_msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
