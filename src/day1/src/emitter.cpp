#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Emitter");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<std_msgs::Int32>("odd", 10);
    ros::Rate rate(100); // 1 Hz

    int num = 42;
   

    while (ros::ok()) {

        std_msgs::Int32 msg;
        msg.data = num;
        
        pub.publish(msg);
        
        rate.sleep();
        
    }

    return 0;
}