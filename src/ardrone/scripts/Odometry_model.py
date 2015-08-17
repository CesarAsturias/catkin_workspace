#!/usr/bin/env python

# @file Odometry_model.py
# @version 1.0
# @date 2015-8-15
# @author Cesr

# Class Odometry_model. Subscribes to navdata topic,
# create the odometry model and publish it
# h = (vx, vy, vz, phi, theta, r)

import rospy
import tf.transformations
from geometry_msgs.msg import Point, Quaternion,  Twist
from ardrone_autonomy.msg import Navdata
from Robot3D import Robot3D
import numpy as np
from math import cos, sin, pi
from nav_msgs.msg import Odometry


class Odometry_model(Robot3D):
    def __init__(self, node_name, rate):
        super(Odometry_model, self).__init__(node_name, rate)

        # Create Point and Quaternion objects
        self.quaternion = Quaternion()
        self.position = Point()
        self.twist = Twist()
        self.odometry = Odometry()
        self.altitude = np.array([0, 0])
        self.yaw = np.array([0, 0])
        self.height = 0
        self.rate = rate
        r = rospy.Rate(self.rate)
        # Subscribes to navdata topic

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.nav_subs = rospy.Subscriber("/ardrone/navdata", Navdata, self.Odometry_callback)
        self.pub = rospy.Publisher('odometry', Odometry, queue_size=10)
        while not rospy.is_shutdown():
            self.pub.publish(self.odometry)
            self.r.sleep()  # With this statement we force the program to loops
            # at 200 Hz

    def Odometry_callback(self, navdata):
        # This function generates an odometry msg from navdata topic
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()  # It should be 5ms

        # Planar velocities in world frame
        vx_world_obs = (navdata.vx * cos(navdata.rotZ * pi / 180) - navdata.vy * sin(navdata.rotZ * pi / 180)) / 1000
        vy_world_obs = (navdata.vx * sin(navdata.rotZ * pi / 180) + navdata.vy * cos(navdata.rotZ * pi / 180))
        # This has to be modified in a posterior version, when the height from 
        # the filter
        # will be available. For now, we are diferentiating the height and yaw measurements.
        self.altitude[1] = self.altitude[0]  # displace the array
        self.altitude[0] = navdata.altd
        vz_obs = ((self.altitude[0] - self.altitude[1])/dt)/1000  # m/s
        self.height = vz_obs * dt + self.height  # Height of the quadrotor. Integration of vz
        self.yaw[1] = self.yaw[0]
        self.yaw[0] = navdata.rotZ
        dot_yaw_obs = ((self.yaw[0] - self.yaw[1])/dt)  # degrees/s
        # Roll and Pitch are direct measurements
        roll_obs = navdata.rotX
        pitch_obs = navdata.rotY
        # Fill quaternion, twist and Point objects and fill Odometry msg
        # quaternion_from_euler returns a numpy ndarray. Maybe we should make a
        # quaternion msg from it
        self.quaternion = tf.transformations.quaternion_from_euler(roll_obs, pitch_obs, navdata.rotZ)
        self.twist.linear.x = vx_world_obs
        self.twist.linear.y = vy_world_obs
        self.twist.linear.z = vz_obs
        self.twist.angular.z = dot_yaw_obs
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.odometry.pose.pose.orientation.x = self.quaternion[0]
        self.odometry.pose.pose.orientation.y = self.quaternion[1]
        self.odometry.pose.pose.orientation.z = self.quaternion[2]
        self.odometry.pose.pose.orientation.w = self.quaternion[3]
        self.odometry.twist.twist = self.twist
        self.odometry.header.seq = navdata.header.seq
        self.odometry.header.frame_id = '/nav'
        self.odometry.child_frame_id = '/nav'
        self.odometry.header.stamp.secs = navdata.header.stamp.secs
        self.odometry.header.stamp.nsecs = navdata.header.stamp.nsecs

if __name__ == '__main__':
    try:
        node_name = "Odometry_model"
        rate = 200  # We want to publish at 200 Hz
        Odometry_model(node_name, rate)
    except KeyboardInterrupt:
        print "Shutting down Odometry model node"
