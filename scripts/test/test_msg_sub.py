#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
"""
    订阅方:
        订阅消息

"""
import rospy
from std_msgs.msg import Int32
from functools import partial

class Obj():
    def __init__(self) -> None:
        self.p = 0
        self.extradata = 89
        
def doPerson(p,obj:Obj):
    rospy.loginfo(f"接收extra {obj.extradata} p {p}")

# def doPerson(extradata):
#     def callback(p):
#         rospy.loginfo(f"接收extra{extradata} p {p.data}")
#     return callback



if __name__ == "__main__":
    #1.初始化节点
    rospy.init_node("test_msg_sub")
    obj = Obj()
    #2.创建订阅者对象
    # 使用partial传递额外参数
    sub = rospy.Subscriber("test_msg",Int32,partial(doPerson, obj=obj),queue_size=10)
    # 使用自带的callback_args传递额外参数
    # sub = rospy.Subscriber("test_msg",Int32,doPerson,queue_size=10,callback_args=obj)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rate.sleep()
        obj.extradata += 1
        rospy.loginfo("test_msg_sub")
        # rospy.spin() #4.循环