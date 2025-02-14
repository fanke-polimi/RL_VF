#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import yaml
from cam_params import stereoCamera

class StereoDepth:
    def __init__(self):
        # rospy.init_node('stereo_depth_node', anonymous=True)
        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber('/endoscope/raw/left/image_raw', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/endoscope/raw/right/image_raw', Image, self.right_image_callback)
        self.left_image = None
        self.right_image = None
        self.map1_left = None
        self.map2_left = None

        # Load calibration parameters
        # self.load_calibration('calibration.yaml')

        # Initialize stereo matcher
        self.stereo = cv2.StereoSGBM_create(minDisparity=0,
                                            numDisparities=16*5,
                                            blockSize=5,
                                            P1=8*3*5**2,
                                            P2=32*3*5**2,
                                            disp12MaxDiff=1,
                                            uniquenessRatio=10,
                                            speckleWindowSize=100,
                                            speckleRange=32)
        self.config=stereoCamera()

    # def load_calibration(self):
        # with open(filepath, 'r') as file:
        #     calibration_data = yaml.safe_load(file)

        self.camera_matrix_left = self.config.cam_matrix_left
        self.camera_matrix_right = self.config.cam_matrix_right
        self.dist_coeffs_left = self.config.distortion_l
        self.dist_coeffs_right = self.config.distortion_r
        self.R = self.config.R
        self.T = self.config.T

        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            self.camera_matrix_left, self.dist_coeffs_left,
            self.camera_matrix_right, self.dist_coeffs_right,
            (640, 480), self.R, self.T, alpha=0)

        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.camera_matrix_left, self.dist_coeffs_left, self.R1, self.P1, (1080, 1920), cv2.CV_16SC2)
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.camera_matrix_right, self.dist_coeffs_right, self.R2, self.P2, (1080, 1920), cv2.CV_16SC2)

    def left_image_callback(self, msg):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert left image: {e}")
            return
        self.process_images()

    def right_image_callback(self, msg):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert right image: {e}")
            return
        self.process_images()

    def process_images(self):
        if self.left_image is not None and self.right_image is not None:
            if self.left_image.size > 0 and self.right_image.size > 0:
                # print(self.map1_left)
                # Rectify images
                rect_left = cv2.remap(self.left_image, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
                rect_right = cv2.remap(self.right_image, self.map1_right, self.map2_right, cv2.INTER_LINEAR)

                # Convert to grayscale
                gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
                gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

                # Compute disparity map
                disparity = self.stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

                # Compute depth map
                depth_map = cv2.reprojectImageTo3D(disparity, self.Q)

                # Normalize the disparity map for visualization
                disparity_vis = (disparity - disparity.min()) / (disparity.max() - disparity.min())
                disparity_vis = (disparity_vis * 255).astype(np.uint8)

                # Display the disparity and depth map
                cv2.imshow('Disparity', self.left_image)
                print(self.left_image)
                # cv2.imshow('Left Image', rect_left)
                # cv2.imshow('Right Image', rect_right)
                cv2.waitKey(1)
            else:
                rospy.logwarn("Received an empty image frame.")

    # def start(self):
    #     rospy.spin()
    #     cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('stereo_depth_node', anonymous=True)
    endo_depth = StereoDepth()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
