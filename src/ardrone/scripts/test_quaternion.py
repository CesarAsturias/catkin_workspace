#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Quaternion
from tf.transformations import *
def vector_to_quaternion(vector):
    try:
        quaternion = np.append(vector, [0])
        return quaternion
    except ValueError, IndexError:
        rospy.loginfo("The argument must be a 3-vector")




