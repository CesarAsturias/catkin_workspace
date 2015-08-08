#!/usr/bin/env python

#@file ardrone_cmd.py
#@version 1.0
#@date 2015-8-8
#@author Cesar

## Class for sending commands to the ardrone. It inherits from 
# the Robot3D class

import rospy
import numpy as np
import time
from Robot3D import Robot3D

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
                print "x = ", position.x
                rospy.sleep(2)
            except:
                continue

if __name__ == '__main__':
    try:
        node_name = "Ardrone"
        Ardrone(node_name)
    except KeyboardInterrupt:
        print "Shutting down ardrone node"

            


