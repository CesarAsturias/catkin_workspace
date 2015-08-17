#!/usr/bin/env python

## @file Robot3D.py
# @version 1.0
# @date 2015-8-7
# @author Cesar

##
# Generic class for a robot moving in 3D. It implements several methods
# for state estimation, transform coordinates and so on.

import rospy
import numpy as np
import tf
import time
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import matplotlib
import matplotlib.pyplot as plt
from ardrone_autonomy.msg import Navdata
from nav_msgs.msg import Odometry


class Robot3D(object):
    def __init__(self, node_name, rate):
        self.node_name = node_name

        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))

        rospy.on_shutdown(self.cleanup)

        # Rate at we will update
        self.rate = rate

        # ROS rate variable
        self.r = rospy.Rate(self.rate)

        # Create a tf.TransformListener object
        self.tf_listener = tf.TransformListener()

        # Create a Publisher object for publish target pose in /base_link frame
        self.pub_target_pose = rospy.Publisher('target_pose', PoseStamped, queue_size = 10)

        # Time
        self.last_time = rospy.Time.now()

        # Subscriber to /ardrone/navdata and publish an Odometry msg with navigation data
        # If the subclass that inherits from Robot3D set navdata, then we have to process
        # the navdata topic
        #if navdata:
         #   odometry = Odometry()
            #self.navdata_subscriber = rospy.Subscriber("/ardrone/navdata", Navdata, self.nav_callback)
            #self.navdata_pub = rospy.Publisher('navdata_topic', Odometry, queue_size = 10)
            # While the node is running, publish the Odometry message at rate frequency
            #while not rospy.is_shutdown():
            #    self.navdata_pub.publish(self.odometry)
            #    r.sleep()

       # Give tf some time to fill its buffer
        rospy.sleep(2)

       #set the nav  frame
        self.nav_frame = '/nav'

       # set the base frame
        self.base_frame = '/base_link'

       # Initialize the position variable as Point type
        position = Point()

    def get_odom(self):
        # Get the current transform between the nav (inertial)
        # and base (body) frames
        # rospy.loginfo("Into get_odom")
        try:
            self.tf_listener.waitForTransform(self.nav_frame, self.base_frame, rospy.Time.now(), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.nav_frame,
                                                            self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), Quaternion(*rot))

    def base_to_nav(self):
        # Obtain the current transform betwen the base (body)
        # and the nav (inertial) frames.
        # @param none
        # @return trans: translation vector
        # @return rot: rotation Quaternion
        try:
            self.tf_listener.waitForTransform(self.nav_frame, self.base_frame, rospy.Time.now(), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame,
                                                            self.nav_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), Quaternion(*rot))

    def create_pose_msg(self, position, orientation, frame):
        # Create a PoseStamped msg, which integrates position and orientation
        # @param position: Point msg containing the position
        # @param orientation: Quaternion msg containing the orientation
        # @param frame: frame of the pose. String ('/base_link', for example)
        # @return pose_stamped: PoseStamped msg

        Pose_stamped = PoseStamped()
        Pose_stamped.pose.position = position
        Pose_stamped.pose.orientation = orientation
        Pose_stamped.header.frame_id = frame
        Pose_stamped.header.stamp = rospy.Time.now()
        return Pose_stamped

    def trans_pose(self, pose, target_frame):
        # Transform the pose in the current frame (PoseStamped msg) to
        # the target_frame
        # @param pose: pose to transform
        # @param target_frame: frame to which we transform the pose (string)
        # @return transformed_pose: PoseStamped message
        return self.tf_listener.transformPose(target_frame, pose)

    def vector_to_quaternion(self, vector):
        # Transform a vector into a Quaternion object(geometry_msgs)
        # The quaternion convention in tf is q = [qx qy qz qw]. Converting
        # a vector to a quaternion consists simply in adding the last com-
        # ponent, qw = 0, to the vector
        # @param vector: numpy array
        # @return Quaternion
        try:
            quaternion = np.append(vector, [0])
            return quaternion
        except ValueError:
            rospy.loginfo("The argument must be a 3-vector")

    def cleanup(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)

# If the file is made executable, then run it. Otherwise, use it as a module
if __name__ == '__main__':
    try:
        Robot3D()
    except:
        rospy.loginfo("Robot3D node terminated")
