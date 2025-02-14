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



def publish_camera_stream():
    rospy.init_node('elp', anonymous=True)
    img_left_pub = rospy.Publisher('/elp/img_left', Image, queue_size=1)
    img_right_pub = rospy.Publisher('/elp/img_right', Image, queue_size=1)
    bridge = CvBridge()

    # 打开USB相机
    CAMERA_NUMBER = 2   


    cap = cv2.VideoCapture(CAMERA_NUMBER)

    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M','J','P','G') )

    #maximum resolution please!
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)
   

    if not cap.isOpened():
        rospy.logerr("无法打开相机")
        return

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        imagePair = cv2.flip(frame,-1)  #I like the USB cable pointing down... 
    
            
        height,width,chaneel = imagePair.shape
        imageL = imagePair[0:height, 0:width//2]
        imageR = imagePair[0:height, width//2:width]
        if not ret:
            rospy.logerr("无法读取相机帧")
            continue

        # 将OpenCV图像转换为ROS图像消息
        image_left_msg = bridge.cv2_to_imgmsg(imageL, "bgr8")
        image_right_msg = bridge.cv2_to_imgmsg(imageR, "bgr8")

        # 发布图像消息
        img_left_pub.publish(image_left_msg)
        img_right_pub.publish(image_right_msg)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_camera_stream()
    except rospy.ROSInterruptException:
        pass
