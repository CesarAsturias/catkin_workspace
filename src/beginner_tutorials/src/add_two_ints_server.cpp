/**
 * @file add_two_ints_server.cpp
 * @author César
 * Example of creating a service and a client. ROS tutorials.
 */

#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

/**
 * @brief This function provides the service for adding two
 * ints, it takes in the request and response type defined
 * in the srv file and returns a boolean.
 *
 * @param req: request (defined in the srv file)
 * @param res: response (defined in the srv file)
 * @return boolean
 */

bool add(beginner_tutorials::AddTwoInts::Request &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b; /**< The two ints are added and stored in the response*/
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

/**
 * @brief Main function. Service advertised over ROS
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}
