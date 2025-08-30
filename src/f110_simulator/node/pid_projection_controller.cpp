w#define _GNU_SOURCE

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
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>





#include <algorithm>
#include <vector>

// //                       0:       1:       2:      3:
// float mapping[4][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {0, 0.0}};

// float speed_limit = 1.8;
// float angle_limit = 0.3;



float target = 1.27;

float prev_err = 0;
float Kp = 1.0f;
float Kd = 1.0f;

float speed = 3.0;
float angle = 0.0;
float speed_limit = 1.8;
float angle_limit = 0.3;

ros::Publisher command_pub;

// float ts_prev = 0.0, ts_now = 0.0, dt = 0.0;

ros::Time ts_now, ts_prev;
float dt = 0;

// Global vars to hold odometry
double yaw = 0.0;
double vx_body = 0.0;
float steering_angle_f=0.0;

void callback_drive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    // Extract the steering angle from the message
    steering_angle_f = msg->drive.steering_angle;
\
}


void callback_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Velocity in robot frame
    vx_body = msg->twist.twist.linear.x;

    // ROS_INFO("Yaw (rad): %f", yaw);

}

float compute_pd(float min_distance, float* prev_error) {
    // float projected_distance = min_distance - vx_body*cos(yaw);
    float projected_distance = min_distance + vx_body * dt * sin(steering_angle_f);

    float error = target - projected_distance;
    static float smoothed_error = 0.0f;
    float alpha = 0.1f; // Smoothing factor
    smoothed_error = alpha * error + (1 - alpha) * smoothed_error;

    float d_error = (error - *prev_error) / dt;
    *prev_error = error;

    float p_part = Kp * error;
    float d_part = Kd * d_error;
    // std::cout << "Error: " << error << ", Smoothed error: " << smoothed_error << ", Prev err: " << *prev_error << ", P part: " << p_part << ", D part: " << d_part << ", dt is: " << dt << std::endl;
    std::cout << "projected_distance: " << projected_distance << ", Min distance: " << min_distance << ", Vx_body: " << vx_body << ", yaw: " << yaw << ", D part: " << std::endl;

    return p_part + d_part;
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

    // if (u > 0.4f) u = 0.4f;
    // if (u < -0.4f) u = -0.4f;

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
  ros::init(argc, argv, "PID_Controller_with_Projection");

  ros::NodeHandle node;

  command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in1", 100);
  ros::Subscriber sub = node.subscribe("scan", 10, callback_scan);
  ros::Subscriber sub2 = node.subscribe("odom", 10, callback_odom);
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



