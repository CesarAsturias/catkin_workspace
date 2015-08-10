#!/usr/bin/env python 

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion

pose_nav = PoseStamped()
position = Point()
position.x = 0.4
position.y = 0.53
position.z = 1

orientation = Quaternion()
orientation.x = 0
orientation.y = 0
orientation.z = 0.11
orientation.w = 0.99


pose_nav.pose.position = position
pose_nav.pose.orientation = orientation 
pose_nav.header.frame_id = '/nav'
pose_nav.header.stamp = 0

print pose_nav


