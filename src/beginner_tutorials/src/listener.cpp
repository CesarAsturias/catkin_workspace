#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This node receive messages over the ROS system
 */

/**
 * This is the callback function that will be called when a new message has arrived
 * on the chatter topic. The message is pased in a boost shared_ptr, which means 
 * you can store it off if you want, without worrying about being deleted underneath
 * you, and without copying the underlying data.
 */

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    /**
     * Call the ros init() function
     */

    ros::init(argc, argv, "listener");

    /**
     * Initialize the node constructing the NodeHandle.
     */

    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic. THis invokes a call to the ROS master node, which keeps a registry
     * of who is publishing and who is subscribing. Messages are passed to a 
     * callback function, here named chatterCallback. subscribe() returns a Subscriber
     * object that you must hold on until you want to unsubscribe. When all copies of
     * the Subscriber object go out of scope, this callback will automatically be unsubscribed
     * from this topic.
     *
     * The second parameter to the subscribe() function is the size of the message queue. If
     * messages are arriving faster than the are being processed, this is the number of
     * messages that will be buffered up before beginning to throw away the oldest ones
     */

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks. With this version, all callbacks will be called
     * from within this thread (the main one). ros::spin() will exit when Ctrl-C is pressed, or
     * the node is shutdown by the master.
     */

    ros::spin();

    return 0;

}
