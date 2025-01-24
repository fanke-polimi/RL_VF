#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import cv2

def publish_camera_tf():
    rospy.init_node('camera_tf_publisher')
    
    # 创建一个tf广播器
    br = tf.TransformBroadcaster()
    
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        # 这里使用示例的旋转向量和平移向量
        # 请替换为你的PnP算法得到的值
        rvec = np.array([0.03897037,0.29469065,2.98512907])
        tvec = np.array([0.03109278,-0.06052072,-0.00328178])
        
        # 将旋转向量转换为旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # 将旋转矩阵转换为四元数
        quat = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((rotation_matrix, np.array([[0], [0], [0]]))), np.array([0, 0, 0, 1])))
        )
        
        # 发布tf
        br.sendTransform(
            (tvec[0], tvec[1], tvec[2]),
            quat,
            rospy.Time.now(),
            "elp/left_camera",  # 子坐标系
            "ECM"          # 父坐标系
        )
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_tf()
    except rospy.ROSInterruptException:
        pass



# """  
#     静态坐标变换发布方:
#         发布关于 laser 坐标系的位置信息 
#     实现流程:
#         1.导包
#         2.初始化 ROS 节点
#         3.创建 静态坐标广播器
#         4.创建并组织被广播的消息
#         5.广播器发送消息
#         6.spin
# """
# # 1.导包
# import rospy
# import tf2_ros
# import tf
# from geometry_msgs.msg import TransformStamped
# import numpy as np
# from scipy.spatial.transform import Rotation as R


 
# if __name__ == "__main__":
#     # 2.初始化 ROS 节点
#     # 示例旋转向量
#     rot_vec = np.array([-0.04276156, 0.20589225, 3.0996263])  # 示例旋转向量，表示绕 (0.1, 0.2, 0.3) 方向旋转

#     # 将旋转向量转换为旋转矩阵
#     rot = R.from_rotvec(rot_vec)
#     rot_matrix = rot.as_matrix()

#     # Rr = np.array([[-0.99901432, -0.03657827, -0.02514812],
#     #           [0.03293065, -0.99061167, 0.13268041],
#     #           [-0.02976525, 0.13172148, 0.99083979]])
#     # R_transpose = np.transpose(Rr)
#     # rot = R.from_matrix(R_transpose)

#     # 将旋转对象转换为四元数
#     quaternion = rot.as_quat()

#     print("Rotation matrix:")
#     print(quaternion)

#     t = np.array([-0.00123274, -0.00945947, -0.01711493])

#     # result = np.dot(-R_transpose, t)
#     # print(result)


#     rospy.init_node("static_tf_pub_p")
#     # 3.创建 静态坐标广播器
#     broadcaster = tf2_ros.StaticTransformBroadcaster()
#     # 4.创建并组织被广播的消息
#     tfs = TransformStamped()
#     # --- 头信息
#     tfs.header.frame_id = "ECM"
#     tfs.header.stamp = rospy.Time.now()
#     tfs.header.seq = 100
#     # --- 子坐标系
#     tfs.child_frame_id = "endoscope_elp/left_camera"
#     # --- 坐标系相对信息
#     # ------ 偏移量
#     tfs.transform.translation.x = 0.08360123
#     tfs.transform.translation.y = -2.04839729
#     tfs.transform.translation.z = 0.85270476
#     # ------ 四元数
#     # qtn = tf.transformations.quaternion_from_euler(0,0,0)
#     #从欧拉角转换成四元数，然后直接对应读取
#     tfs.transform.rotation.x = -1.15893873
#     tfs.transform.rotation.y = 0.89123578
#     tfs.transform.rotation.z = 0.99755523
#     tfs.transform.rotation.w = -0.01741982
 
 
#     # 5.广播器发送消息
#     broadcaster.sendTransform(tfs)
#     # 6.spin
#     rospy.spin()

#     -0.00123274]
#  [-0.00945947]
#  [-0.01711493

# -0.04276156]
#  [ 0.20589225]
#  [ 3.0996263 ]]