#include "ros/ros.h"
#include "std_msgs/Int32.h"

void callback(const std_msgs::Int32::ConstPtr& msg) {
    ROS_INFO("Recv: %d", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Receiver");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("odd", 10, callback);
    ros::spin();
    return 0;
}