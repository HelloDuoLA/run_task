#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped

path = os.path.abspath(".")
sys.path.insert(0, path + "/src/run_task/scripts")

def initialpose_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)

def listener():
    # 初始化节点，命名为'listener'
    rospy.init_node('listener', anonymous=True)

    # 创建一个订阅者，订阅/initialpose话题
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialpose_callback)

    # 保持python程序持续运行直至节点被关闭
    rospy.spin()

if __name__ == '__main__':
    listener()
