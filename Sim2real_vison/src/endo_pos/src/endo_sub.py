#!/usr/bin/env python3

import rospy
import open3d as o3d
import cv2
import numpy as np
# import pcl
from sensor_msgs.msg import Image, PointCloud2,CompressedImage
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from cv_bridge import CvBridge, CvBridgeError
from cam_params import stereoCamera
from depth_utils import preprocess, undistortion, getRectifyTransform, rectifyImage,\
     stereoMatchSGBM

import os
import features
import processor

os.environ["PYOPENGL_PLATFORM"] = "osmesa"


class endo_stereo:
    def __init__(self):
        rospy.init_node('stereo_color_based', anonymous=True)
        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber("/endoscope/raw/left/image_raw/compressed", CompressedImage, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber("/endoscope/raw/right/image_raw/compressed", CompressedImage, self.right_image_callback)
        # self.pointcloud_pub = rospy.Publisher('/camera/point_cloud', PointCloud2, queue_size=10)
        self.left_image = None
        self.right_image = None

        self.pts_l = []
        self.pts_r = []
        self.points_3d = []

        self.orb = cv2.ORB_create()
        self.sift = cv2.xfeatures2d.SIFT_create()
        # self.fast = cv2.FastFeatureDetector_create()

        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

      
        self.config=stereoCamera()

        self.camera_matrix_left = self.config.cam_matrix_left
        self.camera_matrix_right = self.config.cam_matrix_right
        self.dist_coeffs_left = self.config.distortion_l
        self.dist_coeffs_right = self.config.distortion_r
        self.R = self.config.R
        self.T = self.config.T

 


    def left_image_callback(self, msg):
        try:
            self.left_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert left image: {e}")
            return
        self.process_images()

    def right_image_callback(self, msg):

        self.right_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        
        self.process_images()

    def process_images(self):
        if self.left_image is not None and self.right_image is not None:
            if self.left_image.size > 0 and self.right_image.size > 0:
                # folder_path = "/media/fk/Elements11/yolo_dataset/cali/l-6"
                # timeF = 20
                # global count,bridge,img_save1
                # count = count + 1
                # img_save1 = img_save1+1
                # # if count == 1:
                # #     count = 0
                # # cv_img = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
                
                # # cv2.imshow("framel" , cv_img)
                # # cv2.imshow("framelr" , cv_img)
                # cv2.waitKey(1)
                # if(img_save1 % timeF == 0):
                #     print("zuo边",img_save1)
                #     cv2.imwrite(folder_path +"/"+str(int(img_save1/timeF))+".jpg", cv_img)



                map1x, map1y, map2x, map2y, Q = getRectifyTransform(1080, 1920, self.config)
                self.Q = Q
                # iml_rectified, imr_rectified = rectifyImage(self.left_image, self.right_image, map1x, map1y, map2x, map2y)
                iml = undistortion(self.left_image, self.config.cam_matrix_left, self.config.distortion_l)
                imr = undistortion(self.right_image, self.config.cam_matrix_right, self.config.distortion_r)

                

                iml_rectified_l, imr_rectified_r = rectifyImage(iml, imr, map1x, map1y, map2x, map2y)

               
                mask_left = self.detect_red_region(iml_rectified_l)
                mask_right = self.detect_red_region(imr_rectified_r)


                matches1,kp1,kp2, pts1,pts2 = features.find_correspondence_points(iml_rectified_l, imr_rectified_r,mask_left,mask_right)
                # points1 = processor.cart2hom(pt1)
                # points2 = processor.cart2hom(pt2)
                # print(type(pt1))
                print(pts1)
                # print(pt1.shape)
                # print(pt1.T)
                # print(pt1.shape)
                
                # if np.array(self.pts_l).shape[0] < 5:
                #     for pt_left, pt_right in zip(pt1.T, pt2.T):
                #     #     # Draw matched keypointsp
                #         if pt_left[0] not in [x[0] for x in self.pts_l] and pt_right[0] not in [x[0] for x in self.pts_r]:
                #             self.pts_l.append(pt_left)
                #             self.pts_r.append(pt_right)
                            # cv2.circle(iml_rectified_l, (int(pt_left[0]) , int(pt_left[1])), 5, (0, 255, 0), -1)
                            # cv2.circle(imr_rectified_r, (int(pt_right[0]),int(pt_right[1])), 5, (0, 255, 0), -1)
                          

                for pt_left, pt_right in zip(pts1, pts2):
                    # cv2.circle(iml_rectified_l, (int(pt_left[0]) , int(pt_left[1])), 20, (255, 255, 0), -1)
                    # cv2.circle(imr_rectified_r, (int(pt_right[0]),int(pt_right[1])), 20, (255, 255, 0), -1)
                    disparity = pt_left[0] - pt_right[0]
                    depth = self.calculate_depth(disparity)
                    
                    point_3d = self.calculate_3d_coordinates(pt_left, disparity)
                    self.points_3d.append(point_3d[:3])
                    print(disparity)
                    print("shendu",point_3d[:3])
                cv2.circle(iml_rectified_l, (int(pt_left[0]) , int(pt_left[1])), 5, (255, 255, 0), -1)
                cv2.circle(imr_rectified_r, (int(pt_right[0]),int(pt_right[1])), 5, (255, 255, 0), -1)
                matched_image = cv2.drawMatchesKnn(iml_rectified_l, kp1, imr_rectified_r, kp2, list(matches1), None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                iml_, imr_ = preprocess(iml_rectified_l, imr_rectified_r)


                # Display the disparity map
                # cv2.imshow('Disparity', matched_image)
                cv2.imshow('Left Image', iml_rectified_l)
                cv2.imshow('right Image', imr_rectified_r)
                cv2.imshow('right Image_O', self.right_image)
                cv2.imshow('Left mask', self.left_image)
                # cv2.imshow('right mask', self.right_image)
                # cv2.setMouseCallback('Disparity', self.on_mouse_click)
                # print("点击")
                # cv2.imshow('Right Image', rect_right)
                # print(self.left_image)
                cv2.waitKey(1)
            # else:
            #     rospy.logwarn("Received an empty image frame.")

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # if hasattr(self, 'displ'):
                disparity_value = self.filtered_disparity[y, x]
                print(disparity_value)
                depth = self.calculate_depth(disparity_value)
                rospy.loginfo(f"Disparity: {disparity_value}, Depth: {depth} meters")
      

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
        header.frame_id = 'camera'

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

    def detect_red_region(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define red color range
        # lower_red1 = np.array([0, 100, 100])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([160, 100, 100])
        # upper_red2 = np.array([179, 255, 255])
        lower_red1 = np.array([0, 20, 20])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 20, 20])
        upper_red2 = np.array([179, 255, 255])

        # Create a mask for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        # mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

        return mask

    def calculate_depth(self, disparity):
        if disparity > 0:
            # print("TU3",disparity)
            depth = self.Q[2, 3] / (disparity + self.Q[3, 3])
            depth = self.Q[2, 3] / (self.Q[3, 3]-disparity)
            return depth
        else:
            return float('inf')
    def calculate_3d_coordinates(self, pt_left, disparity):
        # Create a disparity image
        disparity_image = np.full((1, 1), disparity, dtype=np.float32)
        # print("TU",disparity_image)
        # # Reproject the points to 3D using the disparity and Q matrix
        # points_3d = cv2.reprojectImageTo3D(disparity_image, self.Q)
        print(self.Q)
        # print("TU2",points_3d)
        # point_3D = points_3d[0, 0]
        point_2D = np.array([pt_left[0], pt_left[1], disparity, 1.0])
        print(point_2D)
        point_3D = np.dot(self.Q, point_2D)
        point_3D /= point_3D[3]
        print(point_3D)
        # point_3D /= disparity # 将齐次坐标转换为3D坐标
        X, Y, Z = point_3D[:3]
        # print(X,Y,Z)

        # points_3d = cv2.reprojectImageTo3D(disparity_image, self.Q)
        # point_3d = point_3d[0][0]

        # # Scale the point coordinates
        # point_3d = point_3d / point_3d[3]

        return point_3D

    def start(self):
        rospy.spin()
        cv2.destroyAllWindows()
        # self.vis.destroy_window()

if __name__ == '__main__':
    stereo_subscriber = endo_stereo()
    stereo_subscriber.start()
