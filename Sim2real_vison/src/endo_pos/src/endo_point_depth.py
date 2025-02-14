#!/usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from cam_params import stereoCamera

# class StereoDepthNode:
#     def __init__(self):
#         rospy.init_node('stereo_depth_node', anonymous=True)
#         self.bridge = CvBridge()
#         self.left_image_sub = rospy.Subscriber('/endoscope/raw/left/image_raw', Image, self.left_image_callback)
#         self.right_image_sub = rospy.Subscriber('/endoscope/raw/right/image_raw', Image, self.right_image_callback)
#         self.left_image = None
#         self.right_image = None

#         # Load stereo camera calibration parameters
#         # self.load_calibration('calibration.yaml')

#         self.config=stereoCamera()

        # self.camera_matrix_left = self.config.cam_matrix_left
        # self.camera_matrix_right = self.config.cam_matrix_right
        # self.dist_coeffs_left = self.config.distortion_l
        # self.dist_coeffs_right = self.config.distortion_r
        # self.R = self.config.R
        # self.T = self.config.T

#     # def load_calibration(self, filepath):
#         # with open(filepath, 'r') as file:
#         #     calibration_data = yaml.safe_load(file)

#         # self.camera_matrix_left = np.array(calibration_data['camera_matrix_left']['data']).reshape(3, 3)
#         # self.camera_matrix_right = np.array(calibration_data['camera_matrix_right']['data']).reshape(3, 3)
#         # self.dist_coeffs_left = np.array(calibration_data['dist_coeffs_left']['data'])
#         # self.dist_coeffs_right = np.array(calibration_data['dist_coeffs_right']['data'])
#         # self.R = np.array(calibration_data['R']['data']).reshape(3, 3)
#         # self.T = np.array(calibration_data['T']['data']).reshape(3, 1)

#         self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
#             self.camera_matrix_left, self.dist_coeffs_left,
#             self.camera_matrix_right, self.dist_coeffs_right,
#             (1920, 1080), self.R, self.T, alpha=0)

#         self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
#             self.camera_matrix_left, self.dist_coeffs_left, self.R1, self.P1, (1920, 1080), cv2.CV_16SC2)
#         self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
#             self.camera_matrix_right, self.dist_coeffs_right, self.R2, self.P2, (1920, 1080), cv2.CV_16SC2)

#     def left_image_callback(self, msg):
#         try:
#             self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         except CvBridgeError as e:
#             rospy.logerr(f"Failed to convert left image: {e}")
#             return
#         self.process_images()

#     def right_image_callback(self, msg):
#         try:
#             self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         except CvBridgeError as e:
#             rospy.logerr(f"Failed to convert right image: {e}")
#             return
#         self.process_images()

#     def process_images(self):
#         if self.left_image is not None and self.right_image is not None:
#             if self.left_image.size > 0 and self.right_image.size > 0:
#                 # Rectify images
#                 rect_left = cv2.remap(self.left_image, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
#                 rect_right = cv2.remap(self.right_image, self.map1_right, self.map2_right, cv2.INTER_LINEAR)

#                 # Detect red points
#                 red_left = self.detect_red_point(rect_left)
#                 red_right = self.detect_red_point(rect_right)

#                 if red_left is not None and red_right is not None:
#                     # Draw circles on the red points
#                     cv2.circle(rect_left, red_left, 5, (0, 0, 255), -1)
#                     cv2.circle(rect_right, red_right, 5, (0, 0, 255), -1)

#                     # Compute disparity and depth
#                     disparity = red_left[0] - red_right[0]
#                     depth = self.calculate_depth(disparity)

#                     rospy.loginfo(f"Red Point Depth: {depth} meters")

#                 # Display the images
#                 cv2.imshow('Left Image', self.left_image)
#                 # cv2.imshow('Right Image', rect_right)
#                 cv2.waitKey(1)
#             else:
#                 rospy.logwarn("Received an empty image frame.")

#     def detect_red_point(self, image):
#         # Convert image to HSV color space
#         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#         # Define red color range
#         lower_red1 = np.array([0, 100, 100])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([160, 100, 100])
#         upper_red2 = np.array([179, 255, 255])

#         # Create a mask for red color
#         mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
#         mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#         mask = cv2.bitwise_or(mask1, mask2)
#         moments = cv2.moments(mask)

#         if moments['m00'] > 0:
#             # Calculate the centroid of the red area
#             cx = int(moments['m10'] / moments['m00'])
#             cy = int(moments['m01'] / moments['m00'])
#             return (cx, cy)
#         else:
#             return None

#     def calculate_depth(self, disparity):
#         if disparity > 0:
#             depth = self.Q[2, 3] / (disparity + self.Q[3, 3])
#             return depth
#         else:
#             return float('inf')

#     def start(self):
#         rospy.spin()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     stereo_depth_node = StereoDepthNode()
#     stereo_depth_node.start()




#########################################################################################################################
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import yaml

class StereoDepthNode:
    def __init__(self):
        rospy.init_node('stereo_depth_node', anonymous=True)
        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber('/endoscope/raw/left/image_raw', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/endoscope/raw/right/image_raw', Image, self.right_image_callback)
        self.left_image = None
        self.right_image = None

        # Load stereo camera calibration parameters
        # self.load_calibration('calibration.yaml')

        # self.config=stereoCamera()

        # self.camera_matrix_left = self.config.cam_matrix_left
        # self.camera_matrix_right = self.config.cam_matrix_right
        # self.dist_coeffs_left = self.config.distortion_l
        # self.dist_coeffs_right = self.config.distortion_r
        # self.R = self.config.R
        # self.T = self.config.T

        # self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
        #     self.camera_matrix_left, self.dist_coeffs_left,
        #     self.camera_matrix_right, self.dist_coeffs_right,
        #     (1920, 1080), self.R, self.T, alpha=0)

        # self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
        #     self.camera_matrix_left, self.dist_coeffs_left, self.R1, self.P1, (1920, 1080), cv2.CV_16SC2)
        # self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
        #     self.camera_matrix_right, self.dist_coeffs_right, self.R2, self.P2, (1920, 1080), cv2.CV_16SC2)

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
                # Rectify images
                # rect_left = cv2.remap(self.left_image, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
                # rect_right = cv2.remap(self.right_image, self.map1_right, self.map2_right, cv2.INTER_LINEAR)

                # # Detect red points
                # red_left, radius_left = self.detect_red_point(rect_left)
                # red_right, radius_right = self.detect_red_point(rect_right)

                # if red_left is not None and red_right is not None:
                #     # Draw circles on the red points
                #     cv2.circle(rect_left, red_left, radius_left, (0, 0, 255), 2)  # Circle with border
                #     cv2.circle(rect_right, red_right, radius_right, (0, 0, 255), 2)  # Circle with border

                #     # Compute disparity and depth
                #     disparity = red_left[0] - red_right[0]
                #     depth = self.calculate_depth(disparity)

                #     # Put text on the image
                #     depth_text = f"Depth: {depth:.2f}m"
                #     cv2.putText(rect_left, depth_text, (red_left[0] - 50, red_left[1] - 30),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                #     cv2.putText(rect_right, depth_text, (red_right[0] - 50, red_right[1] - 30),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                #     rospy.loginfo(f"Red Point Depth: {depth} meters")

                # Display the images
                cv2.imshow('Left Image', self.left_image)
                # cv2.imshow('Right Image', rect_right)
                cv2.waitKey(1)
            else:
                rospy.logwarn("Received an empty image frame.")

    def detect_red_point(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define red color range
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Create a mask for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Get the largest contour
            c = max(contours, key=cv2.contourArea)
            # Get the minimum enclosing circle
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            return (int(cx), int(cy)), int(radius)
        else:
            return None, None

    def calculate_depth(self, disparity):
        if disparity > 0:
            depth = self.Q[2, 3] / (disparity + self.Q[3, 3])
            return depth
        else:
            return float('inf')

    def start(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    stereo_depth_node = StereoDepthNode()
    stereo_depth_node.start()
