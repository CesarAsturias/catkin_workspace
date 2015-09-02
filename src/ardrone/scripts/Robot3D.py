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
from math import pi, cos, sin
from tf import transformations


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
        self.pub_target_pose = rospy.Publisher('target_pose', PoseStamped, queue_size=10)

       # Give tf some time to fill its buffer
        rospy.sleep(2)

       # set the nav  frame
        self.nav_frame = '/nav'

       # set the base frame
        self.base_frame = '/base_link'

       # Initialize the position variable as Point type

       # Some attributes
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        # Constants for the dynamic model
        self.c1 = 0.58
        self.c2 = 17.8
        self.c3 = 10
        self.c4 = 35
        self.c5 = 10
        self.c6 = 25
        self.c7 = 1.4
        self.c8 = 1
        # Initialize commands
        self.command_x = 0
        self.command_y = 0
        self.command_z = 0
        self.command_yaw = 0

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

    def Predict_internal(self, control_command, state):
        # Prediction model. Function callback for the subscriber to cmd_vel topic
        # First, calculate the influence of sent controls:
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        q = state.pose.pose.orientation
        p = state.pose.pose.position
        print type(q)
        print q.x
        rollGain = dt * self.c3 * (self.c4 * control_command.linear.y - q.x)
        pitchGain = dt * self.c3 * (self.c4 * control_command.linear.x - q.y)
        yawspeedGain = dt * self.c5 * (self.c6 * control_command.linear.z - state.twist.twist.angular.z)
        vzGain = dt * self.c7 * (self.c8 * control_command.linear.z - state.twist.twist.linear.z)
        # Update orientation state
        roll = q.x + rollGain
        pitch = q.y + pitchGain
        yaw = q.z + yawspeedGain * dt
        # update twist linear
        yawRad = self.angletorad(q.z)
        rollRad = self.angletorad(q.x)
        pitchRad = self.angletorad(q.y)

        forcex = cos(yawRad) * sin(rollRad) * cos(pitchRad) - sin(yawRad) * sin(pitchRad)
        forcey = -sin(yawRad) * sin(rollRad) * cos(pitchRad) - cos(yawRad) * sin(pitchRad)

        vxGain = dt * (self.c1 * (self.c2 * forcex - state.twist.twist.linear.x))
        vyGain = dt * (self.c1 * (self.c2 * forcey - state.twist.twist.linear.y))

        # State
        state.pose.pose.position.x = state.twist.twist.linear.x * dt + state.pose.pose.position.x
        state.pose.pose.position.y = state.twist.twist.linear.y * dt + state.pose.pose.position.y
        state.pose.pose.position.z = state.twist.twist.linear.z * dt + state.pose.pose.position.z
        # TODO: take account of the takeoff
        quaternion = transformations.quaternion_from_euler(rollRad, pitchRad, yawRad) # It's the correct sequence?
        #state.pose.pose.orientation = quaternion
        state.twist.twist.linear.x = vxGain + state.twist.twist.linear.x
        state.twist.twist.linear.y = vyGain + state.twist.twist.linear.y
        state.twist.twist.linear.z = vzGain + state.twist.twist.linear.z
        state.twist.twist.angular.x = rollGain / dt + state.twist.twist.angular.x
        state.twist.twist.angular.y = pitchGain / dt + state.twist.twist.angular.y
        state.twist.twist.angular.z = yawspeedGain + state.twist.twist.angular.z
        print state
        print type(state)

    def angletorad(self, angle):
        rad = angle * pi / 180
        return rad

    def cleanup(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(0.1)

    def main(args):
        try:
            node_name = "Robot3D"
            rate = 200
            Robot3D(node_name, rate)
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down Robot3D node"

# If the file is made executable, then run it. Otherwise, use it as a module
if __name__ == '__main__':
    main(sys.argv)
