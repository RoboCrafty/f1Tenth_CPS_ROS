#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "random"
#include "cstdlib"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//                       0:       1:       2:      3:
float mapping[4][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {0, 0.0}};

float speed_limit = 1.8;
float angle_limit = 0.3;



std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

unsigned keyToIndex(int message) {
  unsigned res;
  if (message == 1) {
    res = 0;
  }
  else if (message == 2) {
    res = 1;
  }
  else if (message == 3) {
    res = 2;
  }
  else if (message == 4) {
    res = 3;
  }
  else if (message == 5) {
    res = 4;
  }
  return res;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Random_Controller");

  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  
  ros::Rate rate(0.5);  // 2 Hz = every 0.5 sec

  float speed = 0.0;
  float angle = 0.0;

  while(ros::ok()) {
   
    std::srand(std::time(nullptr));
    int input = 0 + (std::rand() % (4 - 0 + 1));


    std::cout << "Number of random value generated: " << input << std::endl;

    unsigned index = keyToIndex(input);
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
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);

    ros::spinOnce();  // process callbacks
    rate.sleep();     // wait for 0.5 sec
  }
  return 0;
}
