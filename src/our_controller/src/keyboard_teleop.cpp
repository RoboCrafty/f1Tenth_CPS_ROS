#define _GNU_SOURCE

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

// Mapping of keys to speed and steering values
// index: 0 = forward, 1 = right, 2 = left, 3 = stop
float mapping[4][2] = {{0.3, 0.0}, {0.3, -1.0}, {0.3, 1.0}, {0, 0.0}};

// Limits for scaling speed and steering angle
float speed_limit = 1.8;
float angle_limit = 0.3;

// Read a single character from keyboard (non-blocking style)
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON; // disable canonical mode
    old.c_lflag &= ~ECHO;   // disable echo
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
    return buf;
}

// Helper to convert std::string to ROS std_msgs::String
std_msgs::String stdStringToRosString(std::string message) {
    std_msgs::String msg;
    msg.data = message;
    return msg;
}

// Global variables for mode and command selection
unsigned res = 3;          // default to stop
std_msgs::Int32 mode;

// Map keyboard input to motion commands and modes
void keyToIndex(char message) {
    if (message == 'f') {
        mode.data = 1; // switch to PID mode
    } else if (message == 'k') {
        mode.data = 2; // switch to keyboard mode
    } else if (message == 'w') {
        res = 0;       // forward
        mode.data = 2;
    } else if (message == 'j') {
        res = 0;       // forward but mode 3
        mode.data = 3;
    } else if (message == 'd') {
        res = 1;       // right
        mode.data = 2;
    } else if (message == 'a') {
        res = 2;       // left
        mode.data = 2;
    } else if (message == 's' || message == ' ') {
        res = 3;       // stop
        mode.data = 2;
    } else {
        mode.data = 2; // stay in keyboard mode
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle n;

    // Publishers: driving commands and mode switching
    ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in2", 1);
    ros::Publisher mode_pub = n.advertise<std_msgs::Int32>("/mode", 1);

    float speed = 0.0;
    float angle = 0.0;

    ros::Rate rate(20); // 20 Hz loop

    while (ros::ok()) {
        char input = getch();    // read key
        keyToIndex(input);       // translate key into action
        unsigned index = res;

        // publish mode (PID, keyboard, etc.)
        mode_pub.publish(mode);

        // fetch speed and angle from mapping table
        speed = mapping[index][0];
        angle = mapping[index][1];

        // Construct Ackermann message
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.speed = speed * speed_limit;
        drive_msg.steering_angle = angle * angle_limit;

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        drive_st_msg.header = header;
        drive_st_msg.drive = drive_msg;

        // Only publish commands in keyboard mode
        if (mode.data == 2) {
            command_pub.publish(drive_st_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
