#!/usr/bin/env python

##@file first_node.py
#@version 1.0 
#@date 2015-8-6
#@author Cesar

#A basic node that subscribes to /ardrone/navdata topic.


import rospy
from ardrone_autonomy.msg import Navdata


## Class Quadrotor
class Quadrotor():
    def __init__(self):
        #Give the node a name
        rospy.init_node('navdata_subscriber', anonymous=False)

        #Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        #Subscriber to /ardrone/navdata
        rospy.Subscriber("/ardrone/navdata", Navdata, self.callback)
        rospy.spin()

    def callback(self, navdata):
        rospy.loginfo("I heard, vx is %f ", navdata.vx)

    def shutdown(self):
        #Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Quadrotor()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished")


