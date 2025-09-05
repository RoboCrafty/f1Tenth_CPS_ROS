#define _GNU_SOURCE

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

/*
 * Gap-Follow Controller
 * ---------------------
 * This ROS node implements a simple "follow-the-gap" algorithm
 * for autonomous driving using LIDAR scan data.
 *
 * Steps:
 *  1. Subscribe to /scan (LaserScan) to process obstacle data.
 *  2. Find the largest gap (open space) in front of the car.
 *  3. Compute a steering angle towards the gap center.
 *  4. Use a PD controller to smooth steering commands.
 *  5. Dynamically scale speed based on obstacle distance ahead.
 *  6. Publish AckermannDriveStamped messages to control the vehicle.
 */

// ---------------- Global Variables ----------------

// PD controller gains
float Kp = 0.60f;
float Kd = 0.12f;
float prev_error = 0.0f;

// Steering variables
float current_steering_angle = 0.0f;   // updated from /drive topic
float steering_multiplier = 1.0f;

// Timing variables for derivative calculation
ros::Time ts_now, ts_prev;
float dt = 0.0f;

// Publisher for drive commands
ros::Publisher command_pub;


// ---------------- Drive Callback ----------------
// Stores the most recent steering angle from the /drive topic
void callback_drive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    current_steering_angle = msg->drive.steering_angle;
    // ROS_INFO("Current steering angle = %f rad", current_steering_angle);
}


// ---------------- Scan Callback ----------------
// Processes LIDAR data, computes steering and speed, and publishes commands
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

    // Save scan data to a CSV file for debugging
    std::ofstream file("scan.csv");
    for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
        file << i << "," << scan_msg->ranges[i] << "\n";
    }
    file.close();

    // Gap-finding parameters
    float threshold = 2.25f;    // minimum safe distance (meters)
    int scan_start = 180;       // ~ -90 degrees
    int scan_end   = 900;       // ~ +90 degrees
    int max_gap_start = -1, max_gap_end = -1, max_gap_size = 0;

    bool in_gap = false;
    int start_idx = -1, end_idx = -1;

    // Find the largest gap in the scan range
    for (int i = scan_start; i < scan_end; ++i) {
        if (scan_msg->ranges[i] > threshold) {
            if (!in_gap) {
                start_idx = i;
                in_gap = true;
            }
        } else {
            if (in_gap) {
                end_idx = i - 1;
                int gap_size = end_idx - start_idx + 1;
                if (gap_size > max_gap_size) {
                    max_gap_size = gap_size;
                    max_gap_start = start_idx;
                    max_gap_end   = end_idx;
                }
                in_gap = false;
            }
        }
    }

    // Handle case where gap extends to the end
    if (in_gap) {
        end_idx = scan_end - 1;
        int gap_size = end_idx - start_idx + 1;
        if (gap_size > max_gap_size) {
            max_gap_size = gap_size;
            max_gap_start = start_idx;
            max_gap_end   = end_idx;
        }
    }

    // Steering angle towards the center of the largest gap
    int gap_center = (max_gap_start + max_gap_end) / 2;
    float steering_angle = (gap_center - 540) * scan_msg->angle_increment; // 540 ≈ straight ahead index

    // PD Controller for steering
    if (ts_prev.isZero()) {
        ts_prev = scan_msg->header.stamp;
        return; // Skip derivative on the first callback
    }

    ts_now = scan_msg->header.stamp;
    ros::Duration dt_duration = ts_now - ts_prev;
    dt = dt_duration.toSec();

    float target = steering_angle; // desired angle
    float error = target - current_steering_angle;

    // Exponential smoothing for error
    static float smoothed_error = 0.0f;
    float alpha = 0.1f;
    smoothed_error = alpha * error + (1 - alpha) * smoothed_error;

    float d_error = (smoothed_error - prev_error) / (1.0 / 40.0f); // assume ~40 Hz loop
    prev_error = smoothed_error;

    float p_part = Kp * smoothed_error;
    float d_part = Kd * d_error;

    steering_angle = p_part + d_part;
    ts_prev = ts_now;

    ROS_INFO("Gap: start=%d, end=%d, center=%d, angle_increment=%f",
             max_gap_start, max_gap_end, gap_center, scan_msg->angle_increment);
    ROS_INFO("Steering_angle = %f rad", steering_angle);

    // Scale forward speed based on distance ahead
    float max_forward_distance = 0.0f;
    for (int i = 510; i <= 570; i++) { // narrow window in front
        if (scan_msg->ranges[i] > max_forward_distance) {
            max_forward_distance = scan_msg->ranges[i];
        }
    }

    float max_speed = 5.5f;
    float k = 0.5f;  // tuning factor
    float safe_speed = max_speed * (1.0f - exp(-k * max_forward_distance));

    // Construct and publish drive command
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = safe_speed;
    drive_msg.steering_angle = steering_angle * steering_multiplier;

    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header.stamp = ros::Time::now();
    drive_st_msg.drive = drive_msg;

    command_pub.publish(drive_st_msg);
}


// ---------------- Main ----------------
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gap_follow_controller");
    ros::NodeHandle node;

    // Publisher for drive commands
    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/mux_in1", 100);

    // Subscribers for LIDAR scan and current steering state
    ros::Subscriber scan_sub  = node.subscribe("scan", 10, callback_scan);
    ros::Subscriber drive_sub = node.subscribe("/drive", 10, callback_drive);

    ros::spin();
    return 0;
}
