#include "ros/ros.h"
#include "std_msgs/Int32.h"

int msg_count = 0;

void callback(const std_msgs::Int32::ConstPtr& msg) {
    msg_count ++; 
    ROS_INFO("Number of message received: %d", msg_count);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Receiver");
    ros::NodeHandle node;

    
    ros::Subscriber sub = node.subscribe("odd", 10, callback);
    ros::spin();
    return 0;
}