#define _GNU_SOURCE

#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <vector>

// Publishers
ros::Publisher command_pub;
ros::Publisher mode_pub;

// Global variables to hold odometry and control states
double yaw = 0.0;          // vehicle yaw
double vx_world_ = 0.0;    // velocity in world x
double vy_world_ = 0.0;    // velocity in world y
double vx_body = 0.0;      // velocity in robot frame
double ttc_threshold = 0.4; // time-to-collision threshold
std_msgs::Bool mode;        // mode message
ros::Time action_start_time;
bool braking = false;       // braking state flag

// Odometry callback: compute yaw and velocities
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    vx_body = msg->twist.twist.linear.x;

    // Convert velocity to world frame
    vx_world_ = vx_body * cos(yaw);
    vy_world_ = vx_body * sin(yaw);
}

// Lidar callback: check for closest obstacle and perform emergency braking
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    if (scan_msg->ranges.empty()) return;

    // Find closest obstacle
    auto it = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end());
    float min_val = *it;
    unsigned int index = std::distance(scan_msg->ranges.begin(), it);

    // Angle of closest point relative to robot
    double angle = scan_msg->angle_min + index * scan_msg->angle_increment;

    // Direction of obstacle in world frame
    double obs_x = cos(yaw + angle);
    double obs_y = sin(yaw + angle);

    // Normalize direction
    double norm = sqrt(obs_x*obs_x + obs_y*obs_y);
    obs_x /= norm;
    obs_y /= norm;

    // Project vehicle velocity onto obstacle direction
    double v_proj = vx_world_ * obs_x + vy_world_ * obs_y;

    if (v_proj > 0.0) {
        double ttc = min_val / v_proj;
        ROS_INFO("Closest obstacle: %.2f m | TTC: %.2f s", min_val, ttc);

        // Trigger braking if TTC below threshold
        if (ttc < ttc_threshold && !braking) {
            braking = true;
            action_start_time = ros::Time::now();
        }

        if (braking) {
            ros::Duration elapsed = ros::Time::now() - action_start_time;

            // Stop reversing after 0.5 seconds
            if (elapsed.toSec() >= 0.5) {
                braking = false;
                ackermann_msgs::AckermannDrive drive_msg;
                drive_msg.speed = 0.0;
                drive_msg.steering_angle = 0.0;
                ackermann_msgs::AckermannDriveStamped drive_st_msg;
                drive_st_msg.header.stamp = ros::Time::now();
                drive_st_msg.drive = drive_msg;
                command_pub.publish(drive_st_msg);

                mode.data = 0;
                mode_pub.publish(mode);
            } else {
                // Keep reversing
                ackermann_msgs::AckermannDrive drive_msg;
                drive_msg.speed = -1.5; // reverse speed
                drive_msg.steering_angle = 0.0;
                ackermann_msgs::AckermannDriveStamped drive_st_msg;
                drive_st_msg.header.stamp = ros::Time::now();
                drive_st_msg.drive = drive_msg;
                command_pub.publish(drive_st_msg);

                // Ensure mode is switched
                mode.data = 0;
                mode_pub.publish(mode);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "AEB");
    ros::NodeHandle node;

    // Publishers
    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in3", 100);
    mode_pub = node.advertise<std_msgs::Bool>("/mode", 1);

    // Subscribers
    ros::Subscriber scan_sub = node.subscribe("scan", 10, callback_scan);
    ros::Subscriber odom_sub = node.subscribe("odom", 10, callback_odom);

    ros::Rate rate(50); // loop rate 50 Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
