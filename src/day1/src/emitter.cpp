#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "iostream"
#include "random"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Emitter");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<std_msgs::Bool>("even", 10);
    ros::Publisher pub2 = node.advertise<std_msgs::Bool>("odd", 10);
    ros::Rate rate(2); // 1 Hz

    // std::random_device rand_gen;
    // std::mt19937 mt(rand_gen());
    // std::uniform_int_distribution distribution(1, 100000);

    int num = 0;
   

    while (ros::ok()) {
        num = rand ();

        std_msgs::Bool msg;
        msg.data = 1;
        
        
        if (num%2==0)
        {
            pub.publish(msg);
        }

        else
        {
            pub2.publish(msg);
        }
        
        
        rate.sleep();
        
    }

    return 0;
}