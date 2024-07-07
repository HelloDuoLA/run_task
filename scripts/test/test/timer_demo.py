#!/usr/bin/env python
# 导入rospy模块
import rospy
import time

# 定时器回调函数
def timer_callback(event):
    rospy.loginfo("Timer callback triggered")
    # time.sleep(2)

# 初始化ROS节点
rospy.init_node('timer_demo_node')

# 创建定时器，设置回调函数和触发间隔（例如，2秒）
timer = rospy.Timer(rospy.Duration(0.2), timer_callback)

# 保持程序运行直到节点被关闭
rospy.spin()