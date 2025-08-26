// #include "ros/ros.h"
// #include "std_msgs/Int32.h"


// int count = 0; 
// int new_data = 0;
// float average = 0;
// float total = 0;

// void callback_arith(const std_msgs::Int32::ConstPtr& msg) {
//     count ++;
//     new_data = msg->data;
//     total = total + new_data;
//     average = total/count;
//     std::cout << "Arithmetic Avg: " << average << std::endl;
// }
// void callback_geo(const std_msgs::Int32::ConstPtr& msg) {
//     count ++;
//     new_data = msg->data;
//     total = total + new_data;
//     average = total/count;
//     std::cout << "Arithmetic Avg: " << average << std::endl;
// }
// void callback_har(const std_msgs::Int32::ConstPtr& msg) {
//     count ++;
//     new_data = msg->data;
//     total = total + new_data;
//     average = total/count;
//     std::cout << "Arithmetic Avg: " << average << std::endl;
// }
// void callback_med(const std_msgs::Int32::ConstPtr& msg) {
//     count ++;
//     new_data = msg->data;
//     total = total + new_data;
//     average = total/count;
//     std::cout << "Arithmetic Avg: " << average << std::endl;
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "Average");
//     ros::NodeHandle node1("Arithmatic Avg"); 
//     ros::NodeHandle node2("Geometric Avg"); 
//     ros::NodeHandle node3("Harmonic Avg"); 
//     ros::NodeHandle node4("Median"); 
//     ros::Subscriber sub1 = node1.subscribe("user_integer", 10, callback_arith);
//     ros::Subscriber sub2 = node1.subscribe("user_integer", 10, callback_geo);
//     ros::Subscriber sub3 = node1.subscribe("user_integer", 10, callback_har);
//     ros::Subscriber sub4 = node1.subscribe("user_integer", 10, callback_med);

//     ros::spin();
//     return 0;
// }

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

int count = 0; 
int new_data = 0;
float average = 0;
float total = 0;
ros::Publisher pub; 

void callback_arith(const std_msgs::Int32::ConstPtr& msg) {
    count ++;
    new_data = msg->data;
    total = total + new_data;
    average = total/count;
    
    std_msgs::Float32 avg_msg;
    avg_msg.data = average;
    pub.publish(avg_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Arith_Average");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback_arith);
    pub = node.advertise<std_msgs::Float32>("arith_avg_pub", 10);

    ros::spin();
    return 0;
}