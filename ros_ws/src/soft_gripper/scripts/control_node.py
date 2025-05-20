#!/usr/bin/env python3
"""
control_node.py
ソフトグリッパー制御用ROSノード雛形
"""
import rospy
from std_msgs.msg import Float32MultiArray

def gripper_force_callback(msg):
    # 受信した力をグリッパーに適用（実装は後続）
    rospy.loginfo(f"Received gripper force: {msg.data}")

if __name__ == "__main__":
    rospy.init_node("soft_gripper_control")
    sub = rospy.Subscriber("/gripper_force", Float32MultiArray, gripper_force_callback)
    rospy.loginfo("soft_gripper_control node started.")
    rospy.spin()
