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

// Desired minimum distance to obstacles
float target = 1.26;

// PD controller parameters
float prev_err = 0;
float Kp = 1.0f;
float Kd = 1.0f;

// Vehicle motion limits
float speed = 2.5;
float angle = 0.0;
float speed_limit = 1.8;
float angle_limit = 0.3;

ros::Publisher command_pub;

// Timing variables for derivative term
ros::Time ts_now, ts_prev;
float dt = 0;

// Compute PD control output based on distance error
float compute_pd(float min_distance, float* prev_error) {
    float error = target - min_distance;

    // Smooth error with low-pass filter
    static float smoothed_error = 0.0f;
    float alpha = 0.1f;  
    smoothed_error = alpha * error + (1 - alpha) * smoothed_error;

    // Derivative term
    float d_error = (smoothed_error - *prev_error) / dt;
    *prev_error = smoothed_error;

    float p_part = Kp * smoothed_error;
    float d_part = Kd * d_error;

    std::cout << "Error: " << error 
              << ", Smoothed: " << smoothed_error 
              << ", P: " << p_part 
              << ", D: " << d_part 
              << ", dt: " << dt 
              << std::endl;

    return p_part + d_part;
}

// Callback for processing laser scans
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Find closest obstacle in scan
    auto it = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end());
    float min_val = *it;
    unsigned int index = std::distance(scan_msg->ranges.begin(), it);

    // Compute time step
    ts_now = scan_msg->header.stamp;
    ros::Duration dt_duration = ts_now - ts_prev;
    dt = dt_duration.toSec();

    // PD controller output
    float u = compute_pd(min_val, &prev_err);

    // Determine steering direction (left/right of center)
    unsigned int center_index = 1080 / 2; // assume 1080 beams
    int direction = (index - center_index);

    float steering_angle = u;
    if (direction > 0) { // obstacle is on the right → steer left
        steering_angle = -steering_angle;
    }

    // Construct and publish Ackermann command
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = steering_angle * angle_limit;

    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;

    command_pub.publish(drive_st_msg);

    std::cout << std::fixed << std::setprecision(10);
    std::cout << "Closest obstacle at: " << min_val 
              << " m, index: " << index 
              << ", u: " << u 
              << ", steering: " << steering_angle 
              << ", direction: " << direction 
              << std::endl;

    ts_prev = ts_now;  // update timestamp
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "PID_Controller");
    ros::NodeHandle node;

    // Publisher: sends steering/speed commands
    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in1", 100);

    // Subscriber: receives LIDAR scans
    ros::Subscriber sub = node.subscribe("scan", 10, callback_scan);

    ros::spin();

    return 0;
}
