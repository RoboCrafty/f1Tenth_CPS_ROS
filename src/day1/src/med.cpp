#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <vector>
#include <algorithm> 
#include "std_msgs/Float32.h"

int count = 0; 
int new_data = 0;
float average = 0;
float total = 0;
std::vector<int> nums;
ros::Publisher pub; 

void callback_med(const std_msgs::Int32::ConstPtr& msg) {
    nums.push_back(msg->data);

    std::vector<int> sorted = nums;
    std::sort(sorted.begin(), sorted.end());
    
    int n = sorted.size();
    double median;
    if (n % 2 == 1) {
        median = sorted[n/2];
    } else {
        median = (sorted[n/2 - 1] + sorted[n/2]) / 2.0;
    }

    std_msgs::Float32 avg_msg;
    avg_msg.data = median;
    pub.publish(avg_msg);
}
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "Med");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback_med);
    pub = node.advertise<std_msgs::Float32>("med_pub", 10);

    ros::spin();
    return 0;
}