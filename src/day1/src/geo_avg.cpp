#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "cmath"
#include "std_msgs/Float32.h"


float count = 0; 
int new_data = 0;
float average = 0;
float total = 1;
ros::Publisher pub; 

void callback_geo(const std_msgs::Int32::ConstPtr& msg) {
    count ++;
    new_data = msg->data;
    total = total * new_data;
    average = std::pow(total, 1.0 / count);
    
    std_msgs::Float32 avg_msg;
    avg_msg.data = average;
    pub.publish(avg_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Geo_Average");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback_geo);
    pub = node.advertise<std_msgs::Float32>("geo_avg_pub", 10);

    ros::spin();
    return 0;
}