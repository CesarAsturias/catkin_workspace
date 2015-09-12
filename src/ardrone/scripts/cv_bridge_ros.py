#!/usr/bin/env python

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class CvBridgeROS(object):
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name)

        # To do in shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)

        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("/camera/rgb/image_raw", Image)
        rospy.loginfo("Ready")

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a numpy array
        frame = np.array(frame, dtype=np.uint8)

        # Process the frame using the process_image() function
        display_image = self.process_image(frame)

        # Display the image
        cv2.imshow(self.node_name, display_image)

        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit")

    def process_image(self, frame):
        # Convert to grayscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)

        # Blur the image
        grey = cv2.blur(grey, (7, 7))

        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)

        return edges

    def cleanup(self):
        print "Shutting down vision node"
        cv2.destroyAllWindows()


def main(args):
    try:
        node_name = "cv_bridge_demo"
        CvBridgeROS(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node"
        cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
