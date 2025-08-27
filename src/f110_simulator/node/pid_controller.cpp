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
#include <sensor_msgs/LaserScan.h>




#include <algorithm>
#include <vector>

// //                       0:       1:       2:      3:
// float mapping[4][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {0, 0.0}};

// float speed_limit = 1.8;
// float angle_limit = 0.3;



float target = 1.3;

float prev_err = 0;
float Kp = 5.0f;
float Kd = 0.0f;

float speed = 1.5;
float angle = 0.0;
float speed_limit = 1.8;
float angle_limit = 0.3;

ros::Publisher command_pub;

// float ts_prev = 0.0, ts_now = 0.0, dt = 0.0;

ros::Time ts_now, ts_prev;
float dt = 0;

float compute_pd(float min_distance, float* prev_error) {
    float error = target - min_distance;
    float d_error = (error - *prev_error) / dt;
    *prev_error = error;

    return Kp * error;
    // return Kp * error + Kd * d_error;
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

    float min_val = 0; 
    unsigned int n = scan_msg->ranges.size();
    // ldp = ld_alloc_new(n);

    // Find min element in scan_msg->ranges
    auto it = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end());
    
    
    // // Get timestamp and calc dt
    // ts_now = scan_msg->header.stamp.toNSec();
    // dt = ts_now - ts_prev;
    // ts_prev = ts_now;

    //static ros::Time ts_prev = scan_msg->header.stamp; // initialize first time

    ts_now = scan_msg->header.stamp;
    ros::Duration dt_duration = ts_now - ts_prev;      // safely handles sec + nsec
    dt = dt_duration.toSec();                  // now in seconds

    

    min_val = *it;
    unsigned int index = std::distance(scan_msg->ranges.begin(), it);

    float u = compute_pd(min_val, &prev_err);

    // Map steering angle to +/- 
    unsigned int center_index = 1080 / 2;
    int direction = (index - center_index);

    float steering_angle = u;
    if (direction > 0) {
        steering_angle = -steering_angle;
    }




    // Make and publish message
    //  Header

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = steering_angle * angle_limit;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);

    std::cout << std::fixed << std::setprecision(10);
    std::cout << "Closest obstacle at: " << min_val 
              << " meters, index: " << index 
              << " u is: "<< u 
              << " steering angle is: "<< steering_angle 
              << " direction is: "<< direction << std::endl;

    ts_prev = ts_now;  // update previous timestamp

    
    

}




int main(int argc, char *argv[]) {
  ros::init(argc, argv, "PID_Controller");

  ros::NodeHandle node;

  command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Subscriber sub = node.subscribe("scan", 10, callback_scan);

  ros::spin();

  float speed = 0.0;
  float angle = 0.0;

//   while(ros::ok()) {
   

//     unsigned index = keyToIndex(input);
//     speed = mapping[index][0];
//     angle = mapping[index][1];

//     // Make and publish message
//     //  Header
//     std_msgs::Header header;
//     header.stamp = ros::Time::now();
//     //  Ackermann
//     ackermann_msgs::AckermannDrive drive_msg;
//     drive_msg.speed = speed * speed_limit;
//     drive_msg.steering_angle = angle * angle_limit;
//     //  AckermannStamped
//     ackermann_msgs::AckermannDriveStamped drive_st_msg;
//     drive_st_msg.header = header;
//     drive_st_msg.drive = drive_msg;
//     // publish AckermannDriveStamped message to drive topic
//     command_pub.publish(drive_st_msg);
//   }
  return 0;
}




// #include "ros/ros.h"
// #include "std_msgs/Bool.h"

// int odd_msg_count = 0;
// int even_msg_count = 0;

// void callback_odd(const std_msgs::Bool::ConstPtr& msg) {
//     odd_msg_count ++; 
//     std::cout << "Number of odd messages: " << odd_msg_count
//           << " ---- Number of even messages: " << even_msg_count
//           << std::endl;
// }

// void callback_even(const std_msgs::Bool::ConstPtr& msg) {
//     even_msg_count ++; 
//     std::cout << "Number of odd messages: " << odd_msg_count
//           << " ---- Number of even messages: " << even_msg_count
//           << std::endl;
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "Receiver");
//     ros::NodeHandle node;

    
//     ros::Subscriber sub = node.subscribe("odd", 10, callback_odd);
//     ros::Subscriber sub2 = node.subscribe("even", 10, callback_even);
//     ros::spin();
//     return 0;
// }