#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Quaternion

va = np.array([1, 2, 3])
va = np.append(va, [0])

va_quaternion = Quaternion(str(va[0]), str(va[1]), str(va[2]), str(va[3]))
print va
print va_quaternion

