#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

int count = 0; 
float new_data = 0;
float average = 0;
float total = 0;
ros::Publisher pub; 

void callback_har(const std_msgs::Int32::ConstPtr& msg) {
    count ++;
    new_data = msg->data;
    total = total + (1/new_data) ;
    average = (count)/(total) ;
    
    std_msgs::Float32 avg_msg;
    avg_msg.data = average;
    pub.publish(avg_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Har_Average");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback_har);
    pub = node.advertise<std_msgs::Float32>("har_avg_pub", 10);

    ros::spin();
    return 0;
}