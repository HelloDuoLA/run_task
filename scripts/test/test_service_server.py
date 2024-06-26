#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys

path = os.path.abspath(".")
# 核心
sys.path.insert(0,path + "/src/run_task/scripts")

from run_task.srv import ImageRecRequest,ImageRecResponse,ImageRec

from std_msgs.msg import String

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('test_service_server')

    server = rospy.Service("ImageRec",ImageRec,doReq)
    # 4.回调函数处理请求并产生响应
    # 5.spin 函数
    rospy.spin()
    
        
def doReq(req):
    # 解析提交的数据
    task_type = req.task_type
    rospy.loginfo("task_type = %d",task_type)

    # 创建响应对象，赋值并返回
    # resp = AddIntsResponse()
    # resp.sum = sum
    resp = ImageRecResponse()
    resp.task_type = task_type
    resp.snack_type = [1, 2, 3]
    resp.XYZ_list = [1.0, 2.0, 3.0,4.0,5.0]
    return resp


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
