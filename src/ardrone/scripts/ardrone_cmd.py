#!/usr/bin/env python

# @file ardrone_cmd.py
# @version 1.0
# @date 2015-8-8
# @author Cesar

## Class for sending commands to the ardrone. It inherits from
# the Robot3D class

import rospy
import numpy as np
import time
from Robot3D import Robot3D
from geometry_msgs.msg import Point, Quaternion


class Ardrone(Robot3D):
    ## Whenever we extend a class, we have to initialize the
    #  parent class as well which is done with Python's
    #  super() function as shown in the following line
    def __init__(self, node_name):
        ## We use super becasue maybe we will want to
        ## override some functions of the Robot3D class
        super(Ardrone, self).__init__(node_name)

        ## Get the position and rotation values
        while not rospy.is_shutdown():
            try:
                (position, rotation) = self.get_odom()
                (position_base, rotation_base) = self.base_to_nav()
                # Select pose
                pos = Point()
                rot = Quaternion()
                pos.x = 1
                pos.y = 1
                pos.z = 1
                rot.x = 0
                rot.y = 0
                rot.z = 0
                rot.w = 1
                pose = self.create_pose_msg(pos, rot, '/nav')
                # Transform to /base_link
                pose_base = self.trans_pose(pose, '/base_link')
                print "position_nav", position
                print "position_base", position_base
                print "pose_nav", pose
                print "pose_base", pose_base
                rospy.sleep(2)
            except:
                continue

if __name__ == '__main__':
    try:
        node_name = "Ardrone"
        Ardrone(node_name)
    except KeyboardInterrupt:
        print "Shutting down ardrone node"
