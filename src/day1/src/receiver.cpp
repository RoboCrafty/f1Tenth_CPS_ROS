#include "ros/ros.h"
#include "std_msgs/Bool.h"

int odd_msg_count = 0;
int even_msg_count = 0;

void callback_odd(const std_msgs::Bool::ConstPtr& msg) {
    odd_msg_count ++; 
    std::cout << "Number of odd messages: " << odd_msg_count
          << " ---- Number of even messages: " << even_msg_count
          << std::endl;
}

void callback_even(const std_msgs::Bool::ConstPtr& msg) {
    even_msg_count ++; 
    std::cout << "Number of odd messages: " << odd_msg_count
          << " ---- Number of even messages: " << even_msg_count
          << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Receiver");
    ros::NodeHandle node;

    
    ros::Subscriber sub = node.subscribe("odd", 10, callback_odd);
    ros::Subscriber sub2 = node.subscribe("even", 10, callback_even);
    ros::spin();
    return 0;
}