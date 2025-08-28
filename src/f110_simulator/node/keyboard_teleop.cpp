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
#include <std_msgs/Bool.h>

//                       0:       1:       2:      3:
float mapping[4][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {0, 0.0}};

float speed_limit = 1.8;
float angle_limit = 0.3;

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

unsigned res = 3;
std_msgs::Bool mode;

void keyToIndex(char message) {

  if (message == 'f') {
    mode.data = 1;
  }
  else if (message == 'k') {
    mode.data = 0;
  }
  else if (message == 'w') {
    res = 0;
    mode.data = 0;
  }
  else if (message == 'd') {
    res = 1;
    mode.data = 0;
  }
  else if (message == 'a') {
    res = 2;
    mode.data = 0;
  }
  else if (message == 's') {
    res = 3;
    mode.data = 0;
  }
  else if (message == ' ') {
    res = 3;
    mode.data = 0;
  }
  else {
    mode.data = 0;
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "keyboard_teleop");

  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in2", 1);
  ros::Publisher mode_pub = n.advertise<std_msgs::Bool>("/mode", 1);

  float speed = 0.0;
  float angle = 0.0;

  ros::Rate rate(20);

  while(ros::ok()) {
    char input  = getch();
    keyToIndex(input);
    unsigned index = res;
    mode_pub.publish(mode);
    speed = mapping[index][0];
    angle = mapping[index][1];

    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = angle * angle_limit;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic when in keyboard mode
    if (mode.data == 0) {
      command_pub.publish(drive_st_msg);
    }
    ros::spinOnce();
    rate.sleep();
  
}

return 0;
}
