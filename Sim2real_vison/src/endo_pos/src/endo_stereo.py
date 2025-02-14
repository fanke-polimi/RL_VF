#!/usr/bin/env python3

import rospy
import open3d as o3d
import cv2
import numpy as np
# import pcl
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from cv_bridge import CvBridge, CvBridgeError
from cam_params import stereoCamera
from depth_utils import preprocess, undistortion, getRectifyTransform, rectifyImage,\
     stereoMatchSGBM

import os

os.environ["PYOPENGL_PLATFORM"] = "osmesa"

CAMERA_NUMBER = 2   #Try zero to start, then increase until it works...

# #create two named windows:
# cv2.namedWindow('Input',cv2.WINDOW_NORMAL)
# cv2.namedWindow('Output', cv2.WINDOW_NORMAL)


#open the camera
cap = cv2.VideoCapture(CAMERA_NUMBER)

cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M','J','P','G') )

#maximum resolution please!
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


class endo_stereo:
    def __init__(self):
        # rospy.init_node('stereo_subscriber', anonymous=True)
        # self.bridge = CvBridge()
        ret,frame = cap.read()
        # imagePair = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        imagePair = cv2.flip(frame,-1)  #I like the USB cable pointing down... 
        print("Image size:", imagePair.shape)
        height,width,chaneel = imagePair.shape
        

        #We assume side by side images and do no error checking!
        self.imageL = imagePair[0:height, 0:width//2]
        self.imageR = imagePair[0:height, width//2:width]
        # self.left_image_sub = rospy.Subscriber('/endoscope/raw/left/image_raw', Image, self.left_image_callback)
        # self.right_image_sub = rospy.Subscriber('/endoscope/raw/left/image_raw', Image, self.right_image_callback)
        # self.pointcloud_pub = rospy.Publisher('/camera/point_cloud', PointCloud2, queue_size=10)
        

    
        self.config=stereoCamera()

        self.camera_matrix_left = self.config.cam_matrix_left
        self.camera_matrix_right = self.config.cam_matrix_right
        self.dist_coeffs_left = self.config.distortion_l
        self.dist_coeffs_right = self.config.distortion_r
        self.R = self.config.R
        self.T = self.config.T

        self.process_images()

    def process_images(self):

                self.left_image = self.imageL
                self.right_image = self.imageR

                # left_undistorted = self.undistort_fisheye(self.left_image, self.camera_matrix_left, self.dist_coeffs_left)
                # right_undistorted = self.undistort_fisheye(self.right_image, self.camera_matrix_right, self.dist_coeffs_right)
    
                map1x, map1y, map2x, map2y, Q = getRectifyTransform(1080, 1920, self.config)
                self.Q = Q
                # iml_rectified, imr_rectified = rectifyImage(self.left_image, self.right_image, map1x, map1y, map2x, map2y)
                iml = undistortion(self.left_image, self.config.cam_matrix_left, self.config.distortion_l)
                imr = undistortion(self.right_image, self.config.cam_matrix_right, self.config.distortion_r)
          
                iml_rectified_l, imr_rectified_r = rectifyImage(iml, imr, map1x, map1y, map2x, map2y)

                iml_, imr_ = preprocess(iml_rectified_l, imr_rectified_r)
                displ, dispr, left_matcher= stereoMatchSGBM(iml_, imr_, False) 

                wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
                wls_filter.setLambda(8000.)
                wls_filter.setSigmaColor(1.3)
                # wls_filter.setLRCthresh(24)
                # # wls_filter.setDepthDiscontinuityRadius(3)

                # filtered_disparity = wls_filter.filter(displ, iml_, None, dispr)
                filtered_disparity = displ
                filteredImg = cv2.normalize(src=filtered_disparity, dst=filtered_disparity, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
                filtered_disparity = np.uint8(filteredImg)
                # filtered_disparity = cv2.normalize(filtered_disparity, None, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
                # filtered_disparity = np.uint8(filtered_disparity)

                # # Compute depth map
                # points_3D = cv2.reprojectImageTo3D(displ, Q)
                # colors = cv2.cvtColor(iml_rectified_l, cv2.COLOR_BGR2RGB)

                # self.visualize_point_cloud(output_points, output_colors)
                # self.save_point_cloud(output_points, output_colors, "point_cloud.ply")

                # Normalize the disparity map for visualization
                # disparity_vis = (filtered_disparity - filtered_disparity.min()) / (filtered_disparity.max() - filtered_disparity.min())
                # disparity_vis = (disparity_vis * 255).astype(np.uint8)

                # self.filtered_disparity = filtered_disparity
                # self.Q = Q

                # Display the disparity map
                # cv2.imshow('Disparity', left_undistorted)
                cv2.imshow('Left Image', filtered_disparity)
                cv2.imshow('Left Image1', iml_rectified_l)
                # cv2.setMouseCallback('Disparity', self.on_mouse_click)
                # print("点击")
                # cv2.imshow('Right Image', rect_right)
                # print(self.left_image)
                cv2.waitKey(1)
            

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # if hasattr(self, 'displ'):
                disparity_value = self.filtered_disparity[y, x]
                print(disparity_value)
                depth = self.calculate_depth(disparity_value)
                rospy.loginfo(f"Disparity: {disparity_value}, Depth: {depth} meters")
                # xy = "%d,%d" % (x, y)

                # cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
                # cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                #             1.0, (255, 255, 255), thickness=1)
                # cv2.imshow("image", img)

    def undistort_fisheye(self,image, K, D):
            h, w = image.shape[:2]
            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=1)
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
            undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            return undistorted_image


    def calculate_depth(self, disparity_value):
        if disparity_value > 0:
            depth = self.Q[2, 3] / (disparity_value + self.Q[3, 3])
            # print(depth)
            return depth
        else:
            return float('inf')

    def visualize_point_cloud(self, points, colors):
        # Create Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors / 255.0)  # Convert to float

        self.vis.clear_geometries()
        # o3d.visualization.draw_geometries([point_cloud],width=1920,height=1080)
        self.vis.update_geometry(point_cloud)
        self.vis.poll_events()
        self.vis.update_renderer()

    def publish_point_cloud(self, points, colors):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'elp'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('r', 12, PointField.UINT8, 1),
            PointField('g', 13, PointField.UINT8, 1),
            PointField('b', 14, PointField.UINT8, 1),
        ]

        point_cloud_data = []
        for p, c in zip(points, colors):
            point_cloud_data.append([p[0], p[1], p[2], c[0], c[1], c[2]])

        point_cloud_msg = pc2.create_cloud(header, fields, point_cloud_data)
        self.pointcloud_pub.publish(point_cloud_msg)

    def save_point_cloud(self, points, colors, filename):
        # Create Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors / 255.0)  # Convert to float

        # Save point cloud to file
        o3d.io.write_point_cloud(filename, point_cloud)
        rospy.loginfo(f"Saved point cloud to {filename}")

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
        # mask = cv2.bitwise_or(mask1, mask2)
        mask = mask1 + mask2

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("这里",contours)
  # 绘制圆形边框
      
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

    # def start(self):
    #     # rospy.spin()
    #     cv2.destroyAllWindows()

if __name__ == '__main__':
    while True:
          stereo_subscriber = endo_stereo()
        #   stereo_subscriber.start()

    cap.release()
    cv2.destroyAllWindows()
