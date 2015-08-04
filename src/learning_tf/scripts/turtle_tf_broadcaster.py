#!/usr/bin/env python
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg
"""
@package turtle_tf_broadcaster.py
Broadcasting the pose of robot in ROS
"""
def handle_turtle_pose(msg, turtlename):
    """This function broadcast the turtle's translationa
    and rotation, and publishes it as a transform from 
    frame 'world' to frame 'turtleX'
    """
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                      tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                      rospy.Time.now(),
                      turtlename,
                      "world")

if __name__ == '__main__':
    """This function (node) subscribes to topic "turtleX/pose" and runs function
    handle_turtle_pose on every incoming mesage
    """
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()

