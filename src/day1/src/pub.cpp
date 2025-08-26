#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Publisher");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<std_msgs::String>("Pub", 10);
    ros::Rate loop_rate(5);

    unsigned int i = 0;
    char message[50];
    std_msgs::String msg;

    while (ros::ok()) {
        printf("Enter name: ");
        scanf("%s", message);
        msg.data = message;
        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }

    return 0;
}