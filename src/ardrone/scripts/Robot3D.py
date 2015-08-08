#!/usr/bin/env python

##@file Robot3D.py
#@version 1.0
#@date 2015-8-7
#@author Cesar

##
# Generic class for a robot moving in 3D. It implements several methods
# for state estimation, transform coordinates and so on.

import rospy
import numpy as np
import tf
import time
from geometry_msgs.msg import Point

class Robot3D(object):
    def __init__(self):
        self.node_name =' Robot3D'

        rospy.init_node('Robot3D', anonymous=False)
        rospy.loginfo("Starting node " + str(self.node_name))

        rospy.on_shutdown(self.shutdown)



        # Rate at we will update 
        rate = 20

        # ROS rate variable
        r = rospy.Rate(rate)

        # Create a tf.TransformListener object.
        self.tf_listener = tf.TransformListener()

       # Give tf some time to fill its buffer
        rospy.sleep(2)

       #set the nav  frame
        self.nav_frame = '/nav'

       # set the base frame
        self.base_frame = '/base_link'
       
       # Initialize the position variable as Point type
        position = Point()

       # Get the  position and rotation values
        while not rospy.is_shutdown():
            try:
                (position, rotation) = self.get_odom()               
                print "Position ", position
                print "Rotation ", rotation
            except:
                continue

    def get_odom(self):
        # Get the current transform between the nav (inertial)  and base (body) frames 
        rospy.loginfo("Into get_odom")
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.nav_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        return (trans, rot)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)
# If the file is made executable, then run it. Otherwise, use it as a module
if __name__ == '__main__':
    try:
        Robot3D()
    except:
        rospy.loginfo("Robot3D node terminated")

