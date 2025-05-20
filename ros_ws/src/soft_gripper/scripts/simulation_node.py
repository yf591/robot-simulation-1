#!/usr/bin/env python3
"""
simulation_node.py
シミュレーション管理用ROSノード雛形
"""
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("simulation_manager")
    pub = rospy.Publisher("/simulation_status", String, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish("running")
        rate.sleep()

if __name__ == "__main__":
    main()
