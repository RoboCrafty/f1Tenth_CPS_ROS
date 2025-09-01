#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>

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



float target = 1.0;

float prev_err = 0;
float Kp = 1.0f;
float Kd = 0.3f;

float speed =2.0;
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
    float projected_distance = min_distance + vx_body * 0.2 * sin(yaw);

    float error = target - projected_distance;
    static float smoothed_error = 0.0f;
    float alpha = 0.1f; // Smoothing factor
    smoothed_error = alpha * error + (1 - alpha) * smoothed_error;

    float d_error = (smoothed_error - *prev_error) / dt;
    *prev_error = smoothed_error;

    float p_part = Kp * smoothed_error;
    float d_part = Kd * d_error;
    // std::cout << "Error: " << error << ", Smoothed error: " << smoothed_error << ", Prev err: " << *prev_error << ", P part: " << p_part << ", D part: " << d_part << ", dt is: " << dt << std::endl;


    float steering_angle = p_part + d_part;

    if (error < 0) {
        steering_angle = fabs(steering_angle);
    }
    else if (error > 0) {
        steering_angle = -fabs(steering_angle);
    }
    std::cout << "projected_distance: " << projected_distance << ", error is: " << error << ", steering angle: " << steering_angle << ", yaw: " << sin(yaw) << std::endl;

    
    return steering_angle;
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

    float min_val = 0; 
    // unsigned int n = scan_msg->ranges.size();
    // ldp = ld_alloc_new(n);
    // pick two angles in radians relative to scan
    float angle_a = 3.14/2;     // 90 deg left
    float angle_b = 0.52;     // 20 deg left-forward

    // convert angles to indices
    int index_b = (int)((angle_b - scan_msg->angle_min) / scan_msg->angle_increment);
    int index_a = (int)((angle_a - scan_msg->angle_min) / scan_msg->angle_increment);
    std::cout << "index a: " << index_a << ", index b: " << index_b << ", Vx_body: " << vx_body << ", yaw: " << sin(yaw) << std::endl;

    // get distances at those indices
    float dA = scan_msg->ranges[index_a];
    float dB = scan_msg->ranges[index_b];

    // wall-follow distance estimate
    float alpha = atan2(dA * cos(angle_a - angle_b) - dB,
                        dA * sin(angle_a - angle_b));

    std::cout << "alpha: " << alpha
                << ", dA: " << dA 
                << ", dB: " << dB
    << std::endl;

    float distance_from_wall = dB * cos(alpha);
    
    // // Get timestamp and calc dt
    // ts_now = scan_msg->header.stamp.toNSec();
    // dt = ts_now - ts_prev;
    // ts_prev = ts_now;x

    //static ros::Time ts_prev = scan_msg->header.stamp; // initialize first time

    ts_now = scan_msg->header.stamp;
    ros::Duration dt_duration = ts_now - ts_prev;      // safely handles sec + nsec
    dt = dt_duration.toSec();                  // now in seconds


    float u = compute_pd(distance_from_wall, &prev_err);



    // Make and publish message
    //  Header

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = u * angle_limit;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);

    std::cout << std::fixed << std::setprecision(10);
    std::cout << "Distance from wall: " << distance_from_wall 
          << ", u: " << u << std::endl;

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


  ros::Rate rate(10); // 20 Hz loop rate (adjust as needed)
  while (ros::ok()) {
      // ... your main loop code ...
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}



