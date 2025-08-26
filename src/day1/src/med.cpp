#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <vector>
#include <algorithm> // for std::sort

int count = 0; 
int new_data = 0;
float average = 0;
float total = 0;
std::vector<int> nums;

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

    std::cout << "Median: " << median << std::endl;
}
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "Med");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("user_integer", 10, callback_med);
    ros::spin();
    return 0;
}