#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('elp', anonymous=True)
        self.image_sub = rospy.Subscriber('/elp/img_left', Image, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # 在这里处理图像，例如显示或进行推理
            cv2.imshow("Image from ROS", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        ImageSubscriber()
    except rospy.ROSInterruptException:
        pass
