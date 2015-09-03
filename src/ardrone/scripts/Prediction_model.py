#!/usr/bin/env python

# @file Prediction_model.py
# @version 1.0
# @date 2015-8-17
# @author Cesar

# Class Kalman_Filter. Subscribes to /cmd_vel topic and 'odometry' topic (measurements)
# and implements a EKF.

import rospy
from geometry_msgs.msg import Twist
from Robot3D import Robot3D
import numpy as np
from math import cos, sin, pi
from nav_msgs.msg import Odometry


class Kalman_Filter(Robot3D):
    def __init__(self, node_name, rate):
        super(Kalman_Filter, self).__init__(node_name, rate)

        # Initialization of the state vector: In the parent class
        self.state = Odometry()

        # Create subscriber and publisher object
        self.command_sub = rospy.Subscriber("cmd_vel", Twist, self.Predict_internal, self.state)
        


        #self.odom_sub = rospy.Subscriber("odometry", Odometry, self.Actualization, self.state)
        #self.state_pub = rospy.Publisher('state_filtered', Odometry, queue_size=10)

        #self.predictor_pub = rospy.Publisher('state', Odometry, queue_size=10)
        
        # Make sure we have received data
        #while self.Predicted_odometry.header.stamp.secs == "":
        #    rospy.sleep(0.1)

        #rospy.loginfo(" Start publishing")

        # Begin the main loop
        #while not rospy.is_shutdown():
         #   self.predictor_pub.publish(self.Predicted_odometry)

if __name__ == '__main__':
    try:
        node_name = "Prediction_model"
        rate = 200
        Kalman_Filter(node_name, rate)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Prediction model node"





