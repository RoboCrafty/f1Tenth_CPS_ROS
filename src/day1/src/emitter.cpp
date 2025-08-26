#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "iostream"
#include "random"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Emitter");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<std_msgs::Int32>("odd", 10);
    ros::Rate rate(3); // 1 Hz

    // std::random_device rand_gen;
    // std::mt19937 mt(rand_gen());
    // std::uniform_int_distribution distribution(1, 100000);

    
    int num;
   

    while (ros::ok()) {
        num = rand ();

        std_msgs::Int32 msg;
        msg.data = num;
        
        if (num%2==0)
        {
        }

        else
        {
            pub.publish(msg);
        }
        
        
        rate.sleep();
        
    }

    return 0;
}