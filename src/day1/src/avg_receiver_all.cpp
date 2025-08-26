#include "ros/ros.h"
#include "std_msgs/Float32.h"



float average = 0;


void callback_arith(const std_msgs::Float32::ConstPtr& msg) {

    average = msg->data;
    std::cout << "Arithmetic Avg: " << average << std::endl;
}
void callback_geo(const std_msgs::Float32::ConstPtr& msg) {

    average = msg->data;
    std::cout << "Geometric Avg: " << average << std::endl;
}
void callback_har(const std_msgs::Float32::ConstPtr& msg) {

    average = msg->data;
    std::cout << "Harmonic Avg: " << average << std::endl;
}
void callback_med(const std_msgs::Float32::ConstPtr& msg) {

    average = msg->data;
    std::cout << "Median : " << average << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Average_receiver");

    ros::NodeHandle node;

    ros::Subscriber sub1 = node.subscribe("arith_avg_pub", 10, callback_arith);
    ros::Subscriber sub2 = node.subscribe("geo_avg_pub", 10, callback_geo);
    ros::Subscriber sub3 = node.subscribe("har_avg_pub", 10, callback_har);
    ros::Subscriber sub4 = node.subscribe("med_pub", 10, callback_med);

    ros::spin();
    return 0;
}