#!/usr/bin/env python
# @file sfm.py
# @author Cesar
# @date 12-09-15

import rospy
import cv2
import cv2.cv as cv
from cv_bridge_ros import CvBridgeROS
import scipy


class Sfm_ROS(CvBridgeROS):
    def __init__(self, node_name):
        super(Sfm_ROS, self).__init__(node_name)

        # Initialize a ORB
        #self.detector = cv2.FeatureDetector_create("ORB")
        #self.descriptor = cv2.DescriptorExtractor_create("ORB")
        self.detector = cv2.ORB(100, 2, 10)
        
    def process_image(self, cv_image):
        # Find keypoints and descriptors directly
        kp = self.detector.detect(cv_image, None)
        kp, dsc = self.detector.compute(cv_image, kp)

        # Draw it on the image
        img = cv2.drawKeypoints(cv_image, kp, None, (255, 0, 0), 4)
        
        
        return img


if __name__ == '__main__':
    try:
        node_name = "Structure_from_motion"
        Sfm_ROS(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Structure from motion node"
        cv2.destroyAllWindows()
