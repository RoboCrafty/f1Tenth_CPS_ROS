#include "ros/ros.h"
#include "std_msgs/Int32.h"


int count = 0; 
float new_data = 0;
float average = 0;
float total = 0;


void callback_har(const std_msgs::Int32::ConstPtr& msg) {
    count ++;
    new_data = msg->data;
    total = total + (1/new_data) ;
    average = (count)/(total) ;
    std::cout << "Harmonic Avg: " << average << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Har_Average");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback_har);
    ros::spin();
    return 0;
}