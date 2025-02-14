#!/usr/bin/env python3

import cv2
from ultralytics import YOLO
import rospy
import open3d as o3d
import cv2
import numpy as np
# import pcl
from sensor_msgs.msg import Image, PointCloud2,CompressedImage
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from cv_bridge import CvBridge, CvBridgeError
from cam_params import stereoCamera
from depth_utils import preprocess, undistortion, getRectifyTransform, rectifyImage,\
     stereoMatchSGBM
import os
import torch

import features
import processor
import struct

import threading

CAMERA_NUMBER = 2   


cap = cv2.VideoCapture(CAMERA_NUMBER)

cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M','J','P','G') )

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
model= YOLO(model="/home/fk/opencv_3d/endo/src/endo_pos/src/yolo/pcd_last.pt")


class elp_stereo:
    def __init__(self):
        rospy.init_node('pcd', anonymous=True)
        
        ret,frame = cap.read()
       
        imagePair = cv2.flip(frame,-1)  #I like the USB cable pointing down... 
 
        
        height,width,chaneel = imagePair.shape
        self.pointcloud_pub = rospy.Publisher('/elp/pcd', PointCloud2, queue_size=1)
        self.elpPose_pub = rospy.Publisher('/elp/pose', PoseStamped, queue_size=1)
        self.targPOse_pub = rospy.Publisher('/elp/targ_pose', PoseStamped, queue_size=10)
        # self.ECM_pose_sub = rospy.Subscriber('/dvrk/PSM2/position_cartesian_current', PoseStamped, self.pose_callback)
        #We assume side by side images and do no error checking!
        self.imageL = imagePair[0:height, 0:width//2]
        self.imageR = imagePair[0:height, width//2:width]


        self.pts_l = []
        self.pts_r = []
        self.points_3d = []
        self.targs_3d = []
        self.psm_2d = np.ndarray([0,0])
   
        self.config=stereoCamera()

        self.camera_matrix_left = self.config.cam_matrix_left
        self.camera_matrix_right = self.config.cam_matrix_right
        self.dist_coeffs_left = self.config.distortion_l
        self.dist_coeffs_right = self.config.distortion_r
        self.R = self.config.R
        self.T = self.config.T
        
        self.process_images()
    def pose_callback(self,msg):
        self.psm_position = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]], dtype=np.float32)
        rvec = np.array([-0.06177771087,-0.24935221,-3.09742796], dtype=np.float32).reshape((3, 1))  # 旋转向量
        tvec = np.array([0.031931600516979,-0.06748304034911,-0.00068895686], dtype=np.float32).reshape((3, 1))  # 平移向量        
        self.psm_2d, _ = cv2.projectPoints(self.psm_position, rvec, tvec, self.camera_matrix_left, self.dist_coeffs_left)
        self.psm_2d = self.psm_2d.ravel() 
    
        # 投影3D点到图像平面
    def process_images(self):   
                
                
                ###############
                map1x, map1y, map2x, map2y, Q = getRectifyTransform(1080, 1920, self.config)
                self.Q = Q
              
                iml = undistortion(self.imageL, self.config.cam_matrix_left, self.config.distortion_l)
                imr = undistortion(self.imageR, self.config.cam_matrix_right, self.config.distortion_r)

                

                iml_rectified_l, imr_rectified_r = rectifyImage(iml, imr, map1x, map1y, map2x, map2y)

                # print(self.imageL.shape)
                cropped_l = iml_rectified_l[250:830, 500:1420]

                cropped_r = imr_rectified_r[250:830, 500:1420]
                ################targ detect
                # with torch.no_grad():
                results_l = model(cropped_l,verbose=False)
                results_r = model(cropped_r,verbose=False)
                annotated_frame = results_l[0].plot()
                annotated_frame_r = results_r[0].plot()
               
                if results_l[0].boxes.xywh.shape[0] == results_r[0].boxes.xywh.shape[0] :
                    for i in range(results_l[0].boxes.xywh.shape[0]):
                        # print("yolo_results",results_l[0].boxes.xywh,results_l[0].boxes.xywh[:,:2], results_l[0].boxes.xywh[:,:2][torch.argsort(results_l[0].boxes.xywh[:,0])[i].item(),:])
                        lymph_left_pixel = results_l[0].boxes.xywh[:,:2][torch.argsort(results_l[0].boxes.xywh[:,0])[i].item(),:]
                        lymph_right_pixel = results_r[0].boxes.xywh[:,:2][torch.argsort(results_r[0].boxes.xywh[:,0])[i].item(),:]
                        # print(lymph_left_pixel.shape,lymph_right_pixel[0].item())
                        lymph_disp=(lymph_left_pixel[0].item() - lymph_right_pixel[0].item())
                        lymph_3d = self.calculate_3d_coordinates(lymph_left_pixel.cpu().numpy(), lymph_disp)
                        self.targs_3d.append(lymph_3d[:3])
                      
                        # print("diandiandian",self.targs_3d)
                        self.publish_targ_pose(lymph_3d[:3])
                        cv2.circle(annotated_frame, (int(lymph_left_pixel[0].item()) , int(lymph_left_pixel[1].item())), 5, (0, 255, 0), -1)
                        cv2.circle(annotated_frame_r, (int(lymph_right_pixel[0]) , int(lymph_right_pixel[1])), 5, (0, 255, 0), -1)
                        del lymph_left_pixel,lymph_right_pixel
                        torch.cuda.empty_cache()
                

                ####### PCD reconstruction
                mask_left = self.detect_red_region(cropped_l)
                mask_right = self.detect_red_region(cropped_r)


                matches1,kp1,kp2, pts1,pts2 = features.find_correspondence_points(cropped_l, cropped_r,mask_left,mask_right)
                # print(pts1)
                pts1 = sorted(pts1, key=lambda point: point[0]) ####对点进行排序
                pts2 = sorted(pts2, key=lambda point: point[0])

                for pt_left, pt_right in zip(pts1, pts2):
                   
                    disparity = pt_left[0] - pt_right[0]
                   
                    point_3d = self.calculate_3d_coordinates(pt_left, disparity)
                    self.points_3d.append(point_3d[:3])
                    # print(disparity)
                    # print("shendu",point_3d[:3])
                    cv2.circle(annotated_frame, (int(pt_left[0]) , int(pt_left[1])), 5, (255, 255, 0), -1)
                    cv2.circle(cropped_r, (int(pt_right[0]),int(pt_right[1])), 5, (255, 255, 0), -1)
                matched_image = cv2.drawMatchesKnn(cropped_l, kp1, cropped_r, kp2, list(matches1), None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
         
                self.publish_point_cloud(self.points_3d, cropped_l)
                self.publish_elp_pose()
                

                # cv2.circle(iml_rectified_l, (int(self.psm_2d[0]), int(self.psm_2d[1])), 3, (0, 0, 255), -1)


                cv2.imshow("left", annotated_frame)
                # cv2.imshow("rrrr", annotated_frame_r)
                # cv2.imshow("right", cropped_l)
           
             
                cv2.waitKey(1)

    def publish_point_cloud(self, points, colors):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'elp'
        point_cloud_data = []
        for p, c in zip(points, colors):
            r = int( 255.0)
            g = int( 255.0)
            b = int( 255.0)
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            point_cloud_data.append([p[0], p[1], p[2], rgb])
       
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]
        # point_cloud_msg = pc2.create_cloud_xyz32(header, points)
        point_cloud_msg = pc2.create_cloud(header, fields, point_cloud_data)
        self.pointcloud_pub.publish(point_cloud_msg)
    def publish_elp_pose(self):
        pose_stamped = PoseStamped()       
        # 填充头部信息
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "ECM"
        rvec = np.array([-0.06177771087,-0.24935221,-3.09742796], dtype=np.float32).reshape((3, 1))  # 旋转向量
        tvec = np.array([0.031931600516979,-0.06748304034911,-0.00068895686], dtype=np.float32).reshape((3, 1))  # 平移向量 
        
        # 将旋转向量转换为旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # 将旋转矩阵转换为四元数
        quat = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((rotation_matrix, np.array([[0], [0], [0]]))), np.array([0, 0, 0, 1])))
        )
        
        # 填充位置信息
        pose_stamped.pose.position = Point(tvec[0], tvec[1], tvec[2])  # 示例位置 (x, y, z)
        
        # 填充方向信息 (四元数)
        pose_stamped.pose.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])  # 示例方向 (x, y, z, w)

        # print(pose_stamped)
        self.elpPose_pub.publish(pose_stamped)

    def publish_targ_pose(self,targ_pos):

        pose_stamped = PoseStamped()
        
        # 填充头部信息
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "elp"
        
        # 填充位置信息
        pose_stamped.pose.position = Point(targ_pos[0]/1000, targ_pos[1]/1000, targ_pos[2]/1000)  # 示例位置 (x, y, z)
        
        # 填充方向信息 (四元数)
        pose_stamped.pose.orientation = Quaternion(0,0,0,0)  # 示例方向 (x, y, z, w)

        # print(pose_stamped)
        self.targPOse_pub.publish(pose_stamped)
    


    def detect_red_region(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 120])
        upper_red2 = np.array([179, 255, 255])
        
        # # Create a mask for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

       

        return mask
    
    def calculate_3d_coordinates(self, pt_left, disparity):
        # Create a disparity image
        # disparity_image = np.full((1, 1), disparity, dtype=np.float32)
        # print("TU",disparity_image)
        # # Reproject the points to 3D using the disparity and Q matrix
        # points_3d = cv2.reprojectImageTo3D(disparity_image, self.Q)
        # print(self.Q)
        # print("TU2",points_3d)
        # point_3D = points_3d[0, 0]
        point_2D = np.array([pt_left[0]+500, pt_left[1]+250, disparity, 1.0])
        # print(point_2D)
        point_3D = np.dot(self.Q, point_2D)
        point_3D /= point_3D[3]
        # print(point_3D)
        # # point_3D /= disparity # 将齐次坐标转换为3D坐标
        X, Y, Z = point_3D[:3]
        # print(X,Y,Z)

        # points_3d = cv2.reprojectImageTo3D(disparity_image, self.Q)
        # points_3d = points_3d[0][0]
        # print(points_3d,point_3D)
        # # # Scale the point coordinates
        # # points_3d = points_3d / points_3d[3]

        # #########
        # X = (pt_left[0] - 998.310793867376) * Z / 595.030335934627
        # Y = (pt_left[1] - 524.342417190547) * Z / 594.224429631422
        # print("zheli",X,Y,Z)

        return point_3D
            

  


    # def start(self):
        # rospy.spin()
        
        # self.vis.destroy_window()

if __name__ == '__main__':
    while not rospy.is_shutdown():    
        elp_stereo()
        torch.cuda.empty_cache()
    # cap.release()
    # cv2.destroyAllWindows()



