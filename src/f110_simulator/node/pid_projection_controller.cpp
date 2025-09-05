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

// Desired distance from wall
float target = 1.0;

// PD controller parameters
float prev_err = 0;
float Kp = 1.0f;
float Kd = 0.3f;

// Vehicle limits
float speed = 2.0;
float angle = 0.0;
float speed_limit = 1.8;
float angle_limit = 0.3;

ros::Publisher command_pub;

// Timing variables
ros::Time ts_now, ts_prev;
float dt = 0;

// State from odometry
double yaw = 0.0;     // orientation
double vx_body = 0.0; // forward velocity
float steering_angle_f = 0.0;

// Listen to steering commands
void callback_drive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    steering_angle_f = msg->drive.steering_angle;
    ROS_INFO("Current steering angle = %f rad", steering_angle_f);
}

// Listen to odometry for yaw + velocity
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
}

// PD controller with error projection
float compute_pd(float min_distance, float* prev_error) {
    // Project distance based on motion
    float projected_distance = min_distance + vx_body * 0.2 * sin(yaw);

    float error = target - projected_distance;

    // Smooth error
    static float smoothed_error = 0.0f;
    float alpha = 0.1f;
    smoothed_error = alpha * error + (1 - alpha) * smoothed_error;

    float d_error = (smoothed_error - *prev_error) / dt;
    *prev_error = smoothed_error;

    float p_part = Kp * smoothed_error;
    float d_part = Kd * d_error;
    float steering_angle = p_part + d_part;

    // Flip steering depending on side of wall
    if (error < 0) {
        steering_angle = fabs(steering_angle);
    } else if (error > 0) {
        steering_angle = -fabs(steering_angle);
    }

    std::cout << "projected_distance: " << projected_distance
              << ", error: " << error
              << ", steering: " << steering_angle
              << ", yaw: " << sin(yaw) << std::endl;

    return steering_angle;
}

// Laser scan callback → wall-following logic
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Pick two beams: left (90°), left-forward (20°)
    float angle_a = M_PI / 2;   // 90 deg
    float angle_b = 0.52;       // 30 deg-ish

    int index_b = (int)((angle_b - scan_msg->angle_min) / scan_msg->angle_increment);
    int index_a = (int)((angle_a - scan_msg->angle_min) / scan_msg->angle_increment);

    float dA = scan_msg->ranges[index_a];
    float dB = scan_msg->ranges[index_b];

    // Wall angle estimation
    float alpha = atan2(dA * cos(angle_a - angle_b) - dB,
                        dA * sin(angle_a - angle_b));

    float distance_from_wall = dB * cos(alpha);

    // Time step
    ts_now = scan_msg->header.stamp;
    ros::Duration dt_duration = ts_now - ts_prev;
    dt = dt_duration.toSec();

    float u = compute_pd(distance_from_wall, &prev_err);

    // Build and publish drive command
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = u * angle_limit;

    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;

    command_pub.publish(drive_st_msg);

    std::cout << std::fixed << std::setprecision(10);
    std::cout << "Distance from wall: " << distance_from_wall
              << ", u: " << u << std::endl;

    ts_prev = ts_now;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "PID_Controller_with_Projection");
    ros::NodeHandle node;

    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in1", 100);

    ros::Subscriber sub_scan = node.subscribe("scan", 10, callback_scan);
    ros::Subscriber sub_odom = node.subscribe("odom", 10, callback_odom);
    ros::Subscriber sub_drive = node.subscribe("/drive", 10, callback_drive);

    ros::spin();
    return 0;
}
