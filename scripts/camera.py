#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import rospy
import os
import sys
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import utilis
import run_task.msg as msg
import task

# 摄像头识别节点, 完成摄像头的识别功能

# ?究竟是通过USB获取图像呢还是话题获取图像呢?
# 先按照话题获取图像的方式来做

# 需求
# 需要摄像头坐标系 <---> 机械臂坐标系的转换

# TODO:待验证

# 摄像头控制器
class camera_controller():
    left_camera_controller = None
    right_camera_controller = None
    def __init__(self,id :utilis.device_id) -> None:
        self.camera_id = id
        self.service = self.camera_service(id)
        if id == utilis.Device_id.LEFT:
            self.left_camera_controller = self
        elif id == utilis.Device_id.RIGHT:
            self.right_camera_controller = self
    
    
class recognition_node():
    def __init__(self) -> None:
        # 订阅图像识别需求
        self.sub_request = rospy.Subscriber(utilis.Topic_name.image_recognition_request,msg.ImageRecRequest,self.do_image_rec_request,callback_args=self,queue_size=10)
        # 发布图像识别结果
        self.pub_result = rospy.Publisher(utilis.Topic_name.image_recognition_result,msg.ImageRecResult,queue_size=10)
    
    @staticmethod
    # 图像识别请求回调
    def do_image_rec_request(request:msg.ImageRecRequest,self:recognition_node):
        result = msg.ImageRecResult()
        # 识别零食
        if   request.task_type == task.Task_type.Task_image_rec.SNACK :
            pass
        # 识别杯子和咖啡机位置
        elif request.task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE :
            pass
        # 识别咖啡机开关
        elif request.task_type == task.Task_type.Task_image_rec.COFFEE_MACHIE_SWITCH :
            pass
        
        # 发布结果
        self.pub_result.publish(result)
    
    # 使用yolo识别
    def yolo(self):
        pass
    
    # 识别stag码
    def recognition_stag_code(self):
        pass


def talker():
    # 初始化节点，命名为'camera'
    rospy.init_node('camera')

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("camera")
        # 按照设定的频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
