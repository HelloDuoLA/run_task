#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

# 语音识别节点
# TODO:待完成功能

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('asr')

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("asr")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
