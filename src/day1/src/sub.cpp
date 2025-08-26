#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Recv: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("Pub", 10, callback);
    ros::spin();
    return 0;
}