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
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <vector>


ros::Publisher command_pub;
ros::Publisher mode_pub;

// Global vars to hold odometry
double yaw = 0.0;
double vx_world_ = 0.0;
double vx_body = 0.0;
double vy_world_ = 0.0;
double ttc_threshold = 0.4; // large initial value
std_msgs::Bool mode;
ros::Time action_start_time;
bool braking = false; // global flag

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


    // Convert to world frame
    vx_world_ = vx_body * cos(yaw);
    vy_world_ = vx_body * sin(yaw);
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

    float min_val = 0; 
    unsigned int n = scan_msg->ranges.size();
    // ldp = ld_alloc_new(n);

    // Find min element in scan_msg->ranges
    auto it = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end());
    
    min_val = *it;
    unsigned int index = std::distance(scan_msg->ranges.begin(), it);

    // Angle of closest point relative to robot
    double angle = scan_msg->angle_min + index * scan_msg->angle_increment;

    // Direction of obstacle in world frame
    double obs_x = cos(yaw + angle);
    double obs_y = sin(yaw + angle);

    // Normalize
    double norm = sqrt(obs_x*obs_x + obs_y*obs_y);
    obs_x /= norm;
    obs_y /= norm;

    // Project velocity vector onto obstacle direction
    double v_proj = vx_world_ * obs_x + vy_world_ * obs_y;

    if (v_proj > 0.0) {
        double ttc = min_val / v_proj;
        ROS_INFO("Closest obstacle: %.2f m | TTC: %.2f s", min_val, ttc);
        
        // Start braking only if TTC is below threshold and not already braking
        if (ttc < ttc_threshold && !braking) {
            braking = true;
            action_start_time = ros::Time::now();
        }

        if (braking) {
            ros::Duration elapsed = ros::Time::now() - action_start_time;

            if (elapsed.toSec() >= 0.5) {
                ROS_WARN("0.5 s elapsed, stopping reverse");
                braking = false;

                // Stop the car
                ackermann_msgs::AckermannDrive drive_msg;
                drive_msg.speed = 0;
                drive_msg.steering_angle = 0.0;
                ackermann_msgs::AckermannDriveStamped drive_st_msg;
                drive_st_msg.header.stamp = ros::Time::now();
                drive_st_msg.drive = drive_msg;
                command_pub.publish(drive_st_msg);

                // Switch mode
                mode.data = 0;
                mode_pub.publish(mode);
            } else {
                // Keep reversing
                ackermann_msgs::AckermannDrive drive_msg;
                drive_msg.speed = -1.5;
                drive_msg.steering_angle = 0.0;
                ackermann_msgs::AckermannDriveStamped drive_st_msg;
                drive_st_msg.header.stamp = ros::Time::now();
                drive_st_msg.drive = drive_msg;
                command_pub.publish(drive_st_msg);

                // Switch mode
                mode.data = 0;
                mode_pub.publish(mode);
            }
        }

       
    }
}




int main(int argc, char *argv[]) {
  ros::init(argc, argv, "AEB");

  ros::NodeHandle node;

  command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in3", 100);
  mode_pub = node.advertise<std_msgs::Bool>("/mode", 1);

  ros::Subscriber sub = node.subscribe("scan", 10, callback_scan);
  ros::Subscriber sub2 = node.subscribe("odom", 10, callback_odom);

  ros::spin();




  ros::Rate rate(50); // 20 Hz loop rate (adjust as needed)
  while (ros::ok()) {
      // ... your main loop code ...
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}



