

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




ros::Publisher command_pub;

bool mode_pid = 0; // 0 for keyboard, 1 for PID



void callback_mode(const std_msgs::Bool & msg) {
    mode_pid = msg.data;
    }

void callback_mux1(const ackermann_msgs::AckermannDriveStamped & msg) {
    if (mode_pid) {
    command_pub.publish(msg);   // forward immediately
  }
    
    }

void callback_mux2(const ackermann_msgs::AckermannDriveStamped & msg) {
    if (!mode_pid) {
    command_pub.publish(msg);   // forward immediately
  }
    }

void callback_mux3(const ackermann_msgs::AckermannDriveStamped & msg) {
    
    command_pub.publish(msg);   // forward immediately
    mode_pid = 0; // switch to keyboard mode after AEB
    
    }


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Multiplexer");

  ros::NodeHandle node;

  command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Subscriber mode_sub = node.subscribe("/mode", 1, callback_mode);
  ros::Subscriber sub1 = node.subscribe("/mux_in1", 10, callback_mux1);
  ros::Subscriber sub2 = node.subscribe("/mux_in2", 10, callback_mux2);
  ros::Subscriber sub3 = node.subscribe("/mux_in3", 10, callback_mux3);



  ros::spin();
  return 0;
}




