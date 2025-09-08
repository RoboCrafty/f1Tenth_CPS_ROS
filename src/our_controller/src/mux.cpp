#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int32.h>

// Publisher to forward selected control messages
ros::Publisher command_pub;

// mode_pid decides which input to forward
// 0 = keyboard, 1 = PID, 2 = safety/AEB, 3 = joystick
int mode_pid = 0;


// Callback to update the current mode (from /mode topic)
void callback_mode(const std_msgs::Int32 & msg) {
    mode_pid = msg.data;
    std::cout << "mode_pid: " << mode_pid << std::endl;
}

// Forward mux1 input if in Gap Follower mode
void callback_mux1(const ackermann_msgs::AckermannDriveStamped & msg) {
    if (mode_pid == 1) {
        command_pub.publish(msg);
    }
}

// Forward mux2 input if in mode 2 (keyboard)
void callback_mux2(const ackermann_msgs::AckermannDriveStamped & msg) {
    if (mode_pid == 2) {
        command_pub.publish(msg);
    }
}

// Always forward mux3 input (AEB = emergency braking),
// then switch to mode 2 (keyboard) after activation (Not used on real car)
void callback_mux3(const ackermann_msgs::AckermannDriveStamped & msg) {
    command_pub.publish(msg);
    mode_pid = 2;
}

// Forward mux4 input if in joystick mode
void callback_mux4(const ackermann_msgs::AckermannDriveStamped & msg) {
    if (mode_pid == 3) {
        command_pub.publish(msg);
    }
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Multiplexer");
    ros::NodeHandle node;

    // Publisher for drive commands to real car
    // command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop", 1);\

    // Publisher for drive commands to simulator.
    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

    // Subscribers for mode and control inputs
    ros::Subscriber mode_sub = node.subscribe("/mode", 1, callback_mode);
    ros::Subscriber sub1 = node.subscribe("/mux_in1", 10, callback_mux1);
    ros::Subscriber sub2 = node.subscribe("/mux_in2", 10, callback_mux2);
    ros::Subscriber sub3 = node.subscribe("/mux_in3", 10, callback_mux3);
    ros::Subscriber sub4 = node.subscribe("/vesc/js", 10, callback_mux4);

    ros::spin();
    return 0;
}
