#!/usr/bin/env python
# @file sfm.py
# @author Cesar
# @date 12-09-15

import rospy
import cv2
import cv2.cv as cv
from cv_bridge_ros import CvBridgeROS
import scipy
from cv2 import KeyPoint


class Sfm_ROS(CvBridgeROS):
    def __init__(self, node_name):
        super(Sfm_ROS, self).__init__(node_name)

        # Initialize a ORB
        # self.detector = cv2.FeatureDetector_create("ORB")
        # self.descriptor = cv2.DescriptorExtractor_create("ORB")
        self.detector = cv2.ORB(20, 2, 10)
        self.prev_img = None
        self.prev_keypoints = KeyPoint()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.matches = None
        self.point_train = []
        self.point_query = []

    def process_image(self, cv_image):
        # Find keypoints and descriptors directly
        if self.prev_img is None:
            self.prev_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


        self.img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        self.matches = self.match_images(self.prev_img, self.img)

        # Draw it on the image
        self.img_keypoints = cv2.drawKeypoints(self.img, self.keypoints, None, (255, 0, 0), 4)
        self.img = self.draw_correlation(self.img_keypoints, self.matches,
                                         self.keypoints, self.prev_keypoints)

        # Copy 
        self.prev_img = self.img
        self.prev_keypoints = self.features_deepcopy(self.keypoints)

        return self.img

    def match_images(self, img1, img2):
        # Compute feature matching between two images.
        # @param img1: first image
        # @param img2: last image
        # @return matches: list of DMatch objects
        self.keypoints = self.detector.detect(img2, None)
        self.keypoints, self.descriptor = self.detector.compute(img2, self.keypoints)
        self.prev_keypoints = self.detector.detect(img1, None)
        self.prev_keypoints, self.prev_descriptor = self.detector.compute(img1, self.prev_keypoints)

        self.matches_vector = self.bf.match(self.prev_descriptor, self.descriptor)
        self.matches_vector = sorted(self.matches_vector, key=lambda x: x.distance)
        return self.matches_vector

    def transform_float_int_tuple(self, input_tuple):
        output_tuple = [0, 0]
        if not input_tuple is None:
            for i in range(0, len(input_tuple)):
                output_tuple[i] = int(input_tuple[i])
        else:
            return input_tuple

        return output_tuple

    def features_deepcopy(self, features):
        return[cv2.KeyPoint(x=k.pt[0], y=k.pt[1],
                _size = k.size, _angle = k.angle,
                _response=k.response, _octave=k.octave,
                _class_id=k.class_id) for k in features]

    def draw_correlation(self, img, matches, keypoints, prev_keypoints):
        # Draw a line betwen the last keypoint position 
        # and the new keypoint position (correlated)
        # @param matches: A matcher object (opencv)
        # @param img: image
        # @param keypoints: keypoints of the new frame
        # @param prev_keypoints: keypoints of the old frame
        # @return img: image with lines betwen correlated points
        
        for i in range(0, len(matches)):
            self.idtrain = matches[i].trainIdx
            self.idquery = matches[i].queryIdx
            self.point_train = keypoints[self.idtrain].pt
            self.point_query = keypoints[self.idquery].pt
            self.point_train = self.transform_float_int_tuple(self.point_train)
            self.point_query = self.transform_float_int_tuple(self.point_query)
            cv2.line(img, ((self.point_train[0]), (self.point_train[1])),
                    ((self.point_query[0]), (self.point_query[1])), (255, 0, 0), i)
        return img



if __name__ == '__main__':
    try:
        node_name = "Structure_from_motion"
        Sfm_ROS(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Structure from motion node"
        cv2.destroyAllWindows()
