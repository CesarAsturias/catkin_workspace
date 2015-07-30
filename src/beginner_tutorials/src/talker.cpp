#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system
 */

int main(int argc, char **argv)
{
    /**
     * The ros::init() function initializes the client library of ROS. We should call this function
     * at the beginning of our nodes. The last parameter is an string containing the default name of the
     * node
     */

    ros::init(argc, argv, "talker");

    /**
     * Now , we start the node using the NodeHandle. When the first ROS node is created, it calls the 
     * ros::start() function, and when the last  ros::NodeHandle is destroyed, it calls the ros::shutdown() 
     * function.
     */

    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to publish on a given topic name.
     * THis invokes a call to the ROS master node, which keeps a registry of who is publishing and who is
     * subscribing. After this advertise() call is made, the master node notify anyone who is trying to subscribe
     * to this topic name, and they will in turn negotiate a peer-to-peer connection with this
     * node. advertise() returns a Publisher object which allows you to publish messages on that topic
     * through a call to publish(). Once all copues of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue used for publishing messages. If 
     * messages are published more quickly than we can send them, the number here specifies how many messages
     * to buffer up before throwing some away.
     */

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000); 

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create a unique string
     * for each message.
     */

    int count = 0;

    while (ros::ok()) {

        /*
         * This is a message object. You stuff it with data, and then publish it.
         */

        std_msgs::String msg;

        // Create a string for the data

        std::stringstream ss;
    
        ss << "hello world" << count;

        //assign the string data to ROS message data field

        msg.data = ss.str();

        ROS_INFO("[Talker] I published %s\n", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;

    }

    return 0;

}
