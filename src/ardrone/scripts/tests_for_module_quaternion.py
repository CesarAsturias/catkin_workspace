#!/usr/bin/env python
import numpy as np
import test_quaternion
from geometry_msgs.msg import Quaternion
from tf.transformations import *
from math import pi

# We are going to transform the vector (1, 2, 3) from A to B, which
# is rotated 90 degrees about the xA axis.
vector = np.array([0, 1, 0])
quaternion_vector = test_quaternion.vector_to_quaternion(vector)
print "vector in quaternion form", quaternion_vector
# Here we create the quaternion representing the rotation of 90
# degrees about the x axis 
quaternion = quaternion_about_axis(pi/2, [1, 0, 0])
print "quaternion: ", quaternion

# Rotate vector 90 degrees about x axis
vector_rotated = quaternion_multiply(quaternion_multiply(quaternion, quaternion_vector),
                               quaternion_conjugate(quaternion))
print "vector rotated: ",  vector_rotated

# quaternion to transform vector in frame A to vector with respect to 
# frame B
quaternion_transform = quaternion_about_axis(-pi/2, [1, 0, 0])
print "quaternion_transform: ", quaternion_transform
# Transform vector to frame B
vector_B = quaternion_multiply(quaternion_multiply(quaternion_transform, quaternion_vector),
                                                   quaternion_conjugate(quaternion_transform))
print "vector in frame B", vector_B

# Pose message





