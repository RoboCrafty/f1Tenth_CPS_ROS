#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "cmath"


float count = 0; 
int new_data = 0;
float average = 0;
float total = 1;


void callback_geo(const std_msgs::Int32::ConstPtr& msg) {
    count ++;
    new_data = msg->data;
    total = total * new_data;
    average = std::pow(total, 1.0 / count);
    std::cout << "Geometric Avg: " << average << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Geo_Average");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback_geo);
    ros::spin();
    return 0;
}