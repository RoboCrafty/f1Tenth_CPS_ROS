#include "ros/ros.h"
#include "std_msgs/Int32.h"


int count = 0; 
int new_data = 0;
float average = 0;
float total = 0;

void callback(const std_msgs::Int32::ConstPtr& msg) {
    count ++;
    new_data = msg->data;
    total = total + new_data;
    average = total/count;
    std::cout << "Arithmetic Avg: " << average << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Average");
    ros::NodeHandle node; 
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback);
    ros::spin();
    return 0;
}