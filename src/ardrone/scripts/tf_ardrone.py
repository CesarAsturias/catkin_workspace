#!/usr/bin/env python

import rospy
import math
import tf


if __name__ == '__main__':
    rospy.init_node('Robot3D')
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/nav', '/base_link', rospy.Time(0))
            print "Position %f" , trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


