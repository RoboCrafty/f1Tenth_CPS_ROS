#include "ros/ros.h"
#include "std_msgs/Float32.h"



float average = 0;


double last_arith = 0.0;
double last_geo   = 0.0;
double last_har   = 0.0;
double last_med   = 0.0;

void callback_arith(const std_msgs::Float32::ConstPtr& msg) {
    last_arith = msg->data;
}
void callback_geo(const std_msgs::Float32::ConstPtr& msg) {
    last_geo = msg->data;
}
void callback_har(const std_msgs::Float32::ConstPtr& msg) {
    last_har = msg->data;
}
void callback_med(const std_msgs::Float32::ConstPtr& msg) {
    last_med = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Average_receiver");

    ros::NodeHandle node;

    ros::Subscriber sub1 = node.subscribe("arith_avg_pub", 10, callback_arith);
    ros::Subscriber sub2 = node.subscribe("geo_avg_pub", 10, callback_geo);
    ros::Subscriber sub3 = node.subscribe("har_avg_pub", 10, callback_har);
    ros::Subscriber sub4 = node.subscribe("med_pub", 10, callback_med);

    ros::Rate rate(2.0);  // 2 Hz = every 0.5 sec

    while (ros::ok()) {
        ROS_INFO("Arith: %.2f | Geo: %.2f | Har: %.2f | Med: %.2f",
                 last_arith, last_geo, last_har, last_med);

        ros::spinOnce();  // process callbacks
        rate.sleep();     // wait for 0.5 sec
    }
    return 0;
}