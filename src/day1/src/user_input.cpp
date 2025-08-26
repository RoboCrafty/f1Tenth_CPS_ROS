#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "User_input");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<std_msgs::Int32>("user_integer", 10);
    ros::Rate loop_rate(5);

    unsigned int i = 0;
    
    std_msgs::Int32 msg;

    while (ros::ok()) {
        printf("Enter number: ");
        std::cin >> i;
        msg.data = i;
        ROS_INFO("%d", msg.data);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}