#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import rospy
import os
import sys
import rospkg
import cv2
import numpy as np
import stag
import time
from tf.transformations import euler_matrix
from typing import List

rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import utilis
import run_task.msg as msg
import task
import order


# 摄像头识别节点, 完成摄像头的识别功能


# STag 识别结果
class STag_result():
    def __init__(self,camera_id:utilis.Device_id, stag_id:int, image_xy=[], base_coords=[]) -> None:
        self.camera_id   = camera_id
        self.stag_id     = stag_id
        self.image_xy    = image_xy
        self.base_coords = base_coords

# STag 识别结果列表
class STag_result_list():
    STag_Snack_dict = {
        order.Snack.Snack_id.YIDA.value      : 1,
        order.Snack.Snack_id.GUOSHU.value    : 2,
        order.Snack.Snack_id.CKU.value       : 3,
        order.Snack.Snack_id.RUSUANJUN.value : 4,
        order.Snack.Snack_id.CHENPIDAN.value : 5,
    }
    def __init__(self) -> None:
        self.stag_result_list:List[STag_result] = []
    
    def add(self,stag_result:STag_result):
        self.stag_result_list.append(stag_result)   
    
    # 绑定yolo结果进行输出
    def bind(self,yolo_result_list:YOLO_result_list)->Rec_result_list:
        rec_result_list = Rec_result_list()
        for stag_result in self.stag_result_list:
            for yolo_result in yolo_result_list.yolo_result_list:
                # 判断STag 中心在YOLO检测框内
                if (stag_result.image_xy[0] > yolo_result.left_top_point_xy[0] and stag_result.image_xy[0] < yolo_result.left_top_point_xy[0] + yolo_result.rectangular_length[0]) \
                and (stag_result.image_xy[1] > yolo_result.left_top_point_xy[1] and stag_result.image_xy[1] < yolo_result.left_top_point_xy[1] + yolo_result.rectangular_length[1]):
                    rec_result = Rec_result(stag_result.camera_id, yolo_result.snack_id, stag_result.image_xy, stag_result.base_coords)
                    rec_result_list.rec_result_list.append(rec_result)
        return rec_result_list
    
    # 通过已知STag id 与零食的关系进行绑定 
    def modify(self)->Rec_result_list:
        rec_result_list = Rec_result_list()
        for stag_result in self.stag_result_list:
            rec_result = Rec_result(stag_result.camera_id, self.STag_Snack_dict[stag_result.stag_id], stag_result.image_xy, stag_result.base_coords)
            rec_result_list.rec_result_list.append(rec_result)
        return rec_result_list

# 识别结果
class Rec_result():
    def __init__(self,camera_id:utilis.Device_id, obj_id:order.Snack.Snack_id, image_xy=[], base_coords=[]) -> None:
        self.camera_id = camera_id
        self.obj_id    = obj_id
        self.image_xy  = image_xy
        self.base_coords = base_coords

# 识别结果列表
class Rec_result_list():
    def __init__(self) -> None:
        self.rec_result_list:List[Rec_result] = []
    
    # 转为消息
    def to_msg(self):
        pass
    
    # 融合
    def fuse(self,other_list:Rec_result_list)->Rec_result_list:
        final_result = Rec_result_list()
        return final_result

# YOLO 识别结果
class YOLO_result():
    def __init__(self,camera_id:utilis.Device_id,snack_id:order.Snack.Snack_id, imag_xy = [],rectangular_length=[]) -> None:
        self.camera_id           = camera_id
        self.snack_id            = snack_id
        self.left_top_point_xy   = imag_xy
        self.rectangular_length  = rectangular_length

# YOLO 识别结果列表
class  YOLO_result_list():
    def __init__(self) -> None:
        self.yolo_result_list:List[YOLO_result] = []
    
    # 增加
    def add(self,yolo_result:YOLO_result):
        self.yolo_result_list.append(yolo_result)
    



# 摄像头控制器
class camera_controller():
    left_camera_controller = None
    right_camera_controller = None
    def __init__(self,id :utilis.device_id) -> None:
        self.camera_id = id
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

# STag_rec识别
def STag_rec(tag_size,mtx,distCoeffs,image,libraryHD=11):
    # 假设的三维点 (例如，一个简单的正方形)
    objectPoints = np.array([
        [-tag_size/2, -tag_size/2,  0],
        [tag_size/2 , -tag_size/2,  0],
        [tag_size/2 , tag_size/2 ,  0],
        [-tag_size/2, tag_size/2 ,  0]
    ], dtype=np.float32)
    
    
    (corners_list, ids, rejected_corners_list) = stag.detectMarkers(image, libraryHD)
    
    # draw detected markers with ids
    stag.drawDetectedMarkers(image, corners_list, ids)

    # draw rejected quads without ids with different color
    stag.drawDetectedMarkers(image, rejected_corners_list, border_color=(255, 0, 0))
    
    # 获取图像的尺寸
    height, width = image.shape[:2]

    # 计算图像中心作为圆心坐标
    center_coordinates = (width // 2, height // 2)
    radius = 5  # 圆的半径
    color = (0, 255, 0)  # BGR颜色，这里为绿色
    thickness = -1  # 设置为-1表示填充整个圆

    # 画圆
    cv2.circle(image, center_coordinates, radius, color, thickness)

    timestamp = int(time.time())

    cv2.imwrite(f'/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/log/STag_result/{timestamp}.jpg', image)
    
    stag_result_list =  STag_result_list()
    
    # 对于每个id都要进行位置检测
    with open(f'/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/log/STag_result/{timestamp}.txt', 'a') as file:
        for i, id in enumerate(ids):
            print(f"Index: {i}, ID: {id[0]}")
            file.write(f"Index: {i}, ID: {id[0]}\n")
            imagePoints  = corners_list[i]
            success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs)
            if success:
                stag_result = STag_result(utilis.Device_id.LEFT, id[0], (imagePoints[0] + imagePoints[1])/2, [translationVector[0][0],translationVector[1][0],translationVector[2][0]])
                stag_result_list.add(stag_result)
                file.write(f"平移向量 x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}\n\n\n")
            else:
                print(f"{i} {id} Failed to solve PnP")
    
    return stag_result_list

class Grip_deviation():
    def __init__(self,x,y,z=-999,const_z=-999) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.const_z = const_z

# 获取偏移
def get_deviation(name,z_is_const=False):
    if z_is_const:
        const_z = rospy.get_param(f'~{name}/z_const')
        x_bias  = rospy.get_param(f'~{name}/x_bias')
        y_bias  = rospy.get_param(f'~{name}/y_bias')
        return Grip_deviation(x_bias,y_bias,const_z=const_z)
    else:
        x_bias  = rospy.get_param(f'~{name}/x_bias')
        y_bias  = rospy.get_param(f'~{name}/y_bias')
        z_bias  = rospy.get_param(f'~{name}/z_bias')
        return Grip_deviation(x_bias,y_bias,z_bias)
    
        

# 初始化两个摄像头
def init_camera():
    global left_camera,right_camera
    
    left_camera_id  = 4
    right_camera_id = 6
    left_camera     = cv2.VideoCapture(left_camera_id)
    right_camera    = cv2.VideoCapture(right_camera_id)
    
    
    frame_width     = rospy.get_param(f'~frame_width',  1280)
    frame_height    = rospy.get_param(f'~frame_height', 960)
    
    left_camera.set(cv2.CAP_PROP_FRAME_WIDTH , frame_width)
    left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    
    right_camera.set(cv2.CAP_PROP_FRAME_WIDTH , frame_width)
    right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

# 初始化偏移常量
def init_const():
    global LeftArmGripSnack, LeftArmGripContainer, LeftArmGripTurnOnMachineSwitch, LeftArmGripTurnOFFMachineSwitch
    global RightArmGripSnack, RightArmGripContainer, RightArmGripCup, RightArmWaterCup
    # 获取项偏移
    LeftArmGripSnack                = get_deviation("LeftArmGripSnack")
    LeftArmGripContainer            = get_deviation("LeftArmGripContainer",True)
    LeftArmGripTurnOnMachineSwitch  = get_deviation("LeftArmGripTurnOnMachineSwitch")
    LeftArmGripTurnOFFMachineSwitch = get_deviation("LeftArmGripTurnOFFMachineSwitch")
    RightArmGripSnack               = get_deviation("RightArmGripSnack")
    RightArmGripContainer           = get_deviation("RightArmGripContainer",True)
    RightArmGripCup                 = get_deviation("RightArmGripCup",True)
    RightArmWaterCup                = get_deviation("RightArmWaterCup",True)

def talker():
    # 初始化节点，命名为'camera'
    rospy.init_node('camera')
    
    init_camera()
    init_const()


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
