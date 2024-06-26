#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
"""
    发布方:
        循环发送消息

"""
import rospy
from std_msgs.msg import String, Int32


if __name__ == "__main__":
    #1.初始化 ROS 节点
    rospy.init_node("talker_person_p")
    #2.创建发布者对象
    pub = rospy.Publisher("test_msg",Int32,queue_size=10)
    #3.组织消息
    p = 32

    #4.编写消息发布逻辑
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(p)  #发布消息
        rate.sleep()  #休眠
