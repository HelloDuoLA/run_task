#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from run_task.msg import *

"""
    需求:
        创建两个ROS 节点，服务器和客户端，
        客户端可以向服务器发送目标数据N(一个整型数据)服务器会计算 1 到 N 之间所有整数的和,
        这是一个循环累加的过程，返回给客户端，这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用 action实现。
    流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 action Client 对象
        4.等待服务
        5.组织目标对象并发送
        6.编写回调, 激活、连续反馈、最终响应
        7.spin
"""
from functools import partial

class Obj():
    def __init__(self) -> None:
        self.p = 0
        self.extradata = 89
        

def done_cb(state,result,obj:Obj):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"响应结果:{result.result} extra {obj.extradata}")

def active_cb():
    rospy.loginfo("服务被激活....")


def fb_cb(fb):
    rospy.loginfo("当前进度:%.2f",fb.progress_bar)

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("action_client_p")
    # 3.创建 action Client 对象
    client = actionlib.SimpleActionClient("addInts",AddIntsAction)
    # 4.等待服务
    client.wait_for_server()
    # 5.组织目标对象并发送
    goal_obj = AddIntsGoal()
    goal_obj.num = 10
    obj = Obj()
    client.send_goal(goal_obj,partial(done_cb,obj=obj),active_cb,fb_cb)
    # 6.编写回调, 激活、连续反馈、最终响应
    # 7.spin
    rospy.spin()
