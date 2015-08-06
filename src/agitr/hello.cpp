// This is a ROS version of the standard "hello, world"
//
// This header defines the standard ROS classes
#include <ros/ros.h>

int main (int argc, char **argv) {
    // Initialize the ROS system
    ros::init(argc, argv, "hello_ros");

    // Establish this program as a ROS node
    ros::NodeHandle nh;

    // Send soome output as a log message
    ROS_INFO_STREAM("Hello, ROS!");

}

