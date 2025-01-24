#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

class JointStateRelay:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('psm_control', anonymous=True)

        # 创建一个发布者
        self.joint_state_publisher = rospy.Publisher('/dvrk/PSM2/set_position_goal_joint', JointState, queue_size=10)
        self.jaw_state_publisher = rospy.Publisher('/dvrk/PSM2/set_position_goal_jaw', JointState, queue_size=10)

        # 创建一个订阅者
        rospy.Subscriber('/unity/psmJS', JointState, self.joint_state_callback)
        rospy.Subscriber('/unity/jaw', JointState, self.jaw_state_callback)

    def joint_state_callback(self, msg):
        print(msg)
        if msg != None:
        # 收到消息后直接发布出去
            rospy.loginfo("Received JointState message, relaying it...")
            self.joint_state_publisher.publish(msg)

    def jaw_state_callback(self, msg):
        print(msg)
        if msg != None:
        # 收到消息后直接发布出去
            rospy.loginfo("Received JointState message, relaying it...")
            self.jaw_state_publisher.publish(msg)

    def spin(self):
        # 保持节点运行，直到节点被关闭
        rospy.spin()

if __name__ == '__main__':
    try:
        relay = JointStateRelay()
        relay.spin()
    except rospy.ROSInterruptException:
        pass
