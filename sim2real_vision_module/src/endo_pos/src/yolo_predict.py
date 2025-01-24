#!/usr/bin/env python

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

CAMERA_NUMBER = 0   


cap = cv2.VideoCapture(CAMERA_NUMBER)

cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M','J','P','G') )

#maximum resolution please!
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)

cap.set(cv2.CAP_PROP_FPS, 60)
model= YOLO(model="/home/fk/opencv_3d/endo/src/endo_pos/src/yolo/last.pt")##model inside the class will make the loop very slow!!!!!!!!!!!
elpconfig=stereoCamera()
psm_2d = np.ndarray([0,0])

# def pose_callback(msg):
#     # global psm_2d
#     psm_position = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]], dtype=np.float32)
    
#     rvec = np.array([-0.07771087,-0.26135221,-3.09742796], dtype=np.float32).reshape((3, 1))  # 旋转向量
#     tvec = np.array([0.031600516979,-0.06548304034911,-0.001000895686], dtype=np.float32).reshape((3, 1))  # 平移向量        
#     # print("psm",self.psm_position)
#     # self.psm_position =  np.array([[0.054875, -0.039686, 0.18675]], dtype=np.float32)
#     psm_2d, _ = cv2.projectPoints(psm_position, rvec, tvec, elpconfig.cam_matrix_left, elpconfig.distortion_l)
#     psm_2d = psm_2d.ravel() 
    

def yolo():
    try:
        
        ret,frame = cap.read()
    
        imagePair = cv2.flip(frame,-1)
        
        height,width,chaneel = imagePair.shape
        print(frame.shape)
        imageL = imagePair[0:height, 0:width//2]
        imageR = imagePair[0:height, width//2:width]
        
        # scale_factor = 0.3
        # left_image = cv2.resize(imageL, None, fx=scale_factor, fy=scale_factor)
        # right_image = cv2.resize(imageR, None, fx=scale_factor, fy=scale_factor)
        # # print(left_image.shape)//324,576
        # # gpu_image_l = cv2.cuda_GpuMat()
        # # gpu_image_l.upload(left_image)
        # # gpu_image_r = cv2.cuda_GpuMat()
        # # gpu_image_r.upload(right_image)
        map1x, map1y, map2x, map2y, Q = getRectifyTransform(480, 640, elpconfig)
        
        # iml = undistortion(imageL, elpconfig.cam_matrix_left, elpconfig.distortion_l)
        # imr = undistortion(imageR, elpconfig.cam_matrix_right, elpconfig.distortion_r)

        
        iml_rectified_l, imr_rectified_r = rectifyImage(imageL, imageR, map1x, map1y, map2x, map2y)
        scale_factor = 2
        left_image_s = cv2.resize(iml_rectified_l, None, fx=scale_factor, fy=scale_factor)
        right_image_s = cv2.resize(imr_rectified_r, None, fx=scale_factor, fy=scale_factor)

        cropped_l = iml_rectified_l[250:830, 500:1420]

        cropped_r = imr_rectified_r[250:830, 500:1420]

        # 使用YOLO进行推理
        results = model(left_image_s)
        annotated_frame = results[0].plot()
        # # print(psm_2d)
        # # cv2.circle(iml_rectified_l, (int(psm_2d[0]), int(psm_2d[1])), 3, (0, 0, 255), -1)

        
        cv2.imshow("left", imageL)
        cv2.imshow("left_s", iml_rectified_l)
        
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    # rospy.init_node('yolo_ros_node', anonymous=True)
    # ECM_pose_sub = rospy.Subscriber('/dvrk/PSM2/position_cartesian_current', PoseStamped, pose_callback)
    # rate = rospy.Rate(25)
    while True:
       yolo()
    # rospy.spin()
    cv2.destroyAllWindows()
