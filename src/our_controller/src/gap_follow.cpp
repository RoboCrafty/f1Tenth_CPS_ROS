#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fstream> 
#include <cstddef>

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



// float target = 1.26;

float Kp = 0.55f;
float Kd = 0.05f;
float prev_error = 0;

float speed = 2.05;
float angle = 0.0;
float speed_limit = 1;
float steering_multiplier = 1;
float current_steering_angle = 0.0; // to hold current steering angle from /drive topic

ros::Publisher command_pub;

// float ts_prev = 0.0, ts_now = 0.0, dt = 0.0;

ros::Time ts_now, ts_prev;
float dt = 0;

void callback_drive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    // Extract the steering angle from the message
    current_steering_angle = msg->drive.steering_angle;
    // ROS_INFO("Current_Steering_angle = %f rad", current_steering_angle);
}


void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {


    std::ofstream file("scan.csv");
    for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
        file << i << "," << scan_msg->ranges[i] << "\n";
    }
    file.close();



  // Gap finding parameters
  float threshold = 1.25; // meters, adjust as needed
  int start_idx = -1, end_idx = -1;
  int max_gap_start = -1, max_gap_end = -1, max_gap_size = 0;
  int scan_start = 180, scan_end = 900; // -90 to +90 degrees in front

  // Find the largest gap in the specified range
  bool in_gap = false;
  for (int i = scan_start; i < scan_end; ++i) {
    if (scan_msg->ranges[i] > threshold) {
      if (!in_gap) {
        // Start of a new gap
        start_idx = i;
        in_gap = true;
      }
    } else {
      if (in_gap) {
        // End of current gap
        end_idx = i - 1;
        int gap_size = end_idx - start_idx + 1;
        if (gap_size > max_gap_size) {
          max_gap_size = gap_size;
          max_gap_start = start_idx;
          max_gap_end = end_idx;
        }
        in_gap = false;
      }
    }
  }
  // Handle case where gap goes till the end
  if (in_gap) {
    end_idx = scan_end - 1;
    int gap_size = end_idx - start_idx + 1;
    if (gap_size > max_gap_size) {
      max_gap_size = gap_size;
      max_gap_start = start_idx;
      max_gap_end = end_idx;
    }
  }

  // Pick the center of the largest gap
  int gap_center = (max_gap_start + max_gap_end) / 2;
  float steering_angle = (gap_center - 540) * scan_msg->angle_increment;



// PD controller for steering  
    if (ts_prev.isZero()) {
    ts_prev = scan_msg->header.stamp;
    return; // Skip derivative calculation on first call
    }


    ts_now = scan_msg->header.stamp;
    ros::Duration dt_duration = ts_now - ts_prev;      // safely handles sec + nsec
    dt = dt_duration.toSec();                  // now in seconds


    float target = steering_angle; // desired angle
    

    float error = target - current_steering_angle;
    static float smoothed_error = 0.0f;
    float alpha = 0.1f; // Smoothing factor
    smoothed_error = alpha * error + (1 - alpha) * smoothed_error;

    float d_error = (smoothed_error - prev_error) / (1.0/40.0);
    prev_error = smoothed_error;

    float p_part = Kp * smoothed_error;
    float d_part = Kd * d_error;
    //std::cout << "Error: " << error << ", d error: " << d_error << ", Prev err: " << prev_error << ", P part: " << p_part << ", D part: " << d_part << ", dt is: " << dt << std::endl;

    steering_angle = p_part + d_part;

    ts_prev = ts_now;  // update previous timestamp

  ROS_INFO("Gap: start=%d, end=%d, center=%d, angle_increment=%f", max_gap_start, max_gap_end, gap_center, scan_msg->angle_increment);
  ROS_INFO("Steering_angle = %f rad", steering_angle);

    // Make and publish message
    //  Header

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = steering_angle * steering_multiplier;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);

    std::cout << std::fixed << std::setprecision(10);
    // std::cout << "Closest obstacle at: " << max_val 
    //           << " meters, index: " << index 
    //           << " steering angle is: "<< steering_angle 
    //           << std::endl;


    

    
    

}




int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Gap_Follow_Controller");

  ros::NodeHandle node;

  command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in1", 100);
  ros::Subscriber sub = node.subscribe("scan", 10, callback_scan);
  ros::Subscriber drive_sub = node.subscribe("/drive", 10, callback_drive);


  ros::spin();

  float speed = 0.0;
  float angle = 0.0;


  ros::Rate rate(50); // 20 Hz loop rate (adjust as needed)
  while (ros::ok()) {
      // ... your main loop code ...
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}



