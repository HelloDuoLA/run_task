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
import copy

rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import utilis
import run_task.msg as msg
import run_task.srv as srv
import task
import order
import arm


LOG_DIR = "/home/elephant/xzc_code/ros_ws/src/run_task/log"

# 摄像头识别节点, 完成摄像头的识别功能


# STag 识别结果
class STag_result():
    def __init__(self,camera_id:utilis.Device_id, stag_id:int, image_xy=[], image_coords=[]) -> None:
        self.camera_id    = camera_id
        self.stag_id      = stag_id
        self.image_xy     = image_xy
        self.image_coords = image_coords                 # 图像三维坐标系
        self.base_coords  = copy.deepcopy(image_coords)  # 机械臂基坐标系
        self.obj_id       = -999           # 识别非零食使用

# STag 识别结果列表
class STag_result_list():
    # TODO:值域为STag码, 根据实际修改
    STag_Snack_dict = {
        8 : order.Snack.Snack_id.YIDA.value,
        7 : order.Snack.Snack_id.GUOSHU.value,
        1 : order.Snack.Snack_id.CKU.value,
        4 : order.Snack.Snack_id.RUSUANJUN.value,
        3 : order.Snack.Snack_id.CHENPIDAN.value,
    }
    
    STag_other_dict = {
        9  : task.Task_image_rec.Rec_OBJ_type.CONTAINER.value ,
        12 : task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH.value,
        3  : task.Task_image_rec.Rec_OBJ_type.CUP.value,
        11 : task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value,
    }
    
    
    def __init__(self) -> None:
        self.stag_result_list:List[STag_result] = []
    
    def add(self,stag_result:STag_result):
        self.stag_result_list.append(stag_result)   
    
    # 转为msg
    def to_msg(self,) -> List[msg.ObjPositionWithID]:
        return_list = []
        for stag_result in self.stag_result_list:
            obj_position = msg.ObjPositionWithID()
            obj_position.arm_id        = stag_result.camera_id
            obj_position.obj_id        = stag_result.obj_id
            obj_position.position      = stag_result.base_coords
            obj_position.position_type = arm.PoseType.BASE_COORDS.value
            return_list.append(obj_position)
        return return_list
    
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
    def to_rec_result(self)->Rec_result_list:
        rec_result_list = Rec_result_list()
        for stag_result in self.stag_result_list:
            rec_result = Rec_result(stag_result.camera_id, self.STag_Snack_dict[stag_result.stag_id], stag_result.image_xy, stag_result.base_coords)
            rec_result_list.rec_result_list.append(rec_result)
        return rec_result_list
    
    # 根据任务与左右手对坐标值进行修正
    def modified_position(self,rec_task_type:task.Task_type.Task_image_rec,arm_id:utilis.Device_id,arm_poses):
        # 零食
        if rec_task_type == task.Task_type.Task_image_rec.SNACK:
            # 左臂
            if arm_id == utilis.Device_id.LEFT:
                for i in range(len(self.stag_result_list)):
                    stag_result = self.stag_result_list[i]
                    stag_result.base_coords[0] = arm_poses[0]  +  stag_result.image_coords[2] + LeftArmGripSnack.x    # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1]  -  stag_result.image_coords[1] + LeftArmGripSnack.y    # y = y - y + bias
                    stag_result.base_coords[2] = arm_poses[2]  +  stag_result.image_coords[0] + LeftArmGripSnack.z    # z = z + x + bias
            # 右臂
            elif arm_id == utilis.Device_id.RIGHT:
                for i in range(len(self.stag_result_list)):
                    stag_result = self.stag_result_list[i]
                    stag_result.base_coords[0] = arm_poses[0]  +  stag_result.image_coords[2] + RightArmGripSnack.x   # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1]  +  stag_result.image_coords[1] + RightArmGripSnack.y   # y = y + y + bias
                    stag_result.base_coords[2] = arm_poses[2]  -  stag_result.image_coords[0] + RightArmGripSnack.z   # z = z - x + bias
        # 容器
        elif rec_task_type == task.Task_type.Task_image_rec.CONTAINER:
            if arm_id == utilis.Device_id.LEFT:
                # 左手
                new_stag_result_list = [ ]
                for i in range(len(self.stag_result_list)):
                    stag_result = copy.deepcopy(self.stag_result_list[i]) 
                    # 寻找容器STag
                    if stag_result.stag_id == self.STag_other_dict[task.Task_image_rec.Rec_OBJ_type.CONTAINER]:
                        stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[0] + LeftArmGripContainer.x # x = x + x + bias
                        stag_result.base_coords[1] = arm_poses[1] - stag_result.image_coords[1] + LeftArmGripContainer.y # y = y - y + bias
                        stag_result.base_coords[2] = LeftArmGripContainer.const_z                     #固定z坐标
                        stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.CONTAINER.value
                        new_stag_result_list.append(stag_result)
                self.stag_result_list = new_stag_result_list
            elif arm_id == utilis.Device_id.RIGHT:
                # 右手
                new_stag_result_list = [ ]
                for i in range(len(self.stag_result_list)):
                    stag_result = copy.deepcopy(self.stag_result_list[i]) 
                    # 寻找容器STag
                    if stag_result.stag_id == self.STag_other_dict[task.Task_image_rec.Rec_OBJ_type.CONTAINER]:
                        stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[0] + RightArmGripContainer.x # x = x - x + bias
                        stag_result.base_coords[1] = arm_poses[1] + stag_result.image_coords[1] + RightArmGripContainer.y # y = y + y + bias
                        stag_result.base_coords[2] = RightArmGripContainer.const_z                     #固定z坐标
                        stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.CONTAINER.value
                        new_stag_result_list.append(stag_result)
                self.stag_result_list = new_stag_result_list
        # 咖啡机和杯子
        elif rec_task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE:
            new_stag_result_list = [ ]
            # 只有右手
            for i in range(len(self.stag_result_list)):
                stag_result = copy.deepcopy(self.stag_result_list[i])  
                # 先判断是不是杯子或接水点
                if stag_result.stag_id == self.STag_other_dict[task.Task_image_rec.Rec_OBJ_type.CUP]:
                    # 杯子
                    stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[2] + RightArmGripCup.x  # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1] + stag_result.image_coords[1] + RightArmGripCup.y  # y = y + y + bias
                    stag_result.base_coords[2] = RightArmGripCup.const_z                              #固定z坐标
                    stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.CUP.value
                    new_stag_result_list.append(stag_result)
                elif stag_result.stag_id == self.STag_other_dict[task.Task_image_rec.Rec_OBJ_type.WATER_POINT]:
                    # 接水点
                    stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[2] + RightArmWaterCup.x  # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1] + stag_result.image_coords[1] + RightArmWaterCup.y  # y = y + y + bias
                    stag_result.base_coords[2] = RightArmWaterCup.const_z                             #固定z坐标
                    stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value
                    new_stag_result_list.append(stag_result)
            self.stag_result_list = new_stag_result_list
        # 打开咖啡机开关
        elif rec_task_type == task.Task_type.Task_image_rec.COFFEE_MACHINE_SWITCH_ON:
            # 识别开机, 只有左手
            new_stag_result_list = [ ]
            for i in range(len(self.stag_result_list)):
                stag_result = copy.deepcopy(self.stag_result_list[i])  
                if stag_result.stag_id == self.STag_other_dict[task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH]:
                    stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[2] + LeftArmGripTurnOnMachineSwitch.x  # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1] - stag_result.image_coords[1] + LeftArmGripTurnOnMachineSwitch.y  # y = y - y + bias
                    stag_result.base_coords[2] = arm_poses[2] + stag_result.image_coords[0] + LeftArmGripTurnOnMachineSwitch.z  # z = z + x + bias
                    stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH.value
                    new_stag_result_list.append(stag_result)
            self.stag_result_list = new_stag_result_list
        # 关闭咖啡机开关
        elif rec_task_type == task.Task_type.Task_image_rec.COFFEE_MACHINE_SWITCH_OFF:
            # 识别关机, 只有左手
            new_stag_result_list = [ ]
            for i in range(len(self.stag_result_list)):
                stag_result = copy.deepcopy(self.stag_result_list[i])  
                if stag_result.stag_id == self.STag_other_dict[task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH]:
                    stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[2] + LeftArmGripTurnOFFMachineSwitch.x # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1] - stag_result.image_coords[1] + LeftArmGripTurnOFFMachineSwitch.y # y = y - y + bias
                    stag_result.base_coords[2] = arm_poses[2] + stag_result.image_coords[0] + LeftArmGripTurnOFFMachineSwitch.z # z = z + x + bias
                    stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH.value
                    new_stag_result_list.append(stag_result)
            self.stag_result_list = new_stag_result_list
        else:
            raise ValueError("rec_task_type is not defined")

# 识别结果
class Rec_result():
    def __init__(self,camera_id:utilis.Device_id, obj_id:order.Snack.Snack_id, image_xy=[], base_coords=[]) -> None:
        self.camera_id = camera_id
        self.obj_id    = obj_id
        self.image_xy  = image_xy
        self.base_coords = base_coords
    
    def __str__(self) -> None:
        return f"camera_id : {self.camera_id}\n obj_id : {self.obj_id } \n image_xy : {self.image_xy} \n base_coords : {self.base_coords}"

# 识别结果列表
class Rec_result_list():
    def __init__(self) -> None:
        self.rec_result_list:List[Rec_result] = []
    
    def __str__(self) -> None:
        str_print = ""
        for rec_result in self.rec_result_list:
            str_print += str(rec_result) + "\n"
        return str_print
    
    # 转为消息
    def to_msg(self)->List[msg.ObjPositionWithID]:
        return_list = []
        for stag_result in self.rec_result_list:
            obj_position = msg.ObjPositionWithID()
            obj_position.arm_id        = stag_result.camera_id
            obj_position.obj_id        = stag_result.obj_id
            obj_position.position      = stag_result.base_coords
            obj_position.position_type = arm.PoseType.BASE_COORDS.value
            return_list.append(obj_position)
        return return_list
    
    # 融合
    def fuse(self,left_arm_rec_result_list:Rec_result_list)->Rec_result_list:
        final_result = Rec_result_list()
        processed_ids = set()

        # 处理self.rec_result_list中的结果
        for rec_result in self.rec_result_list:
            snack_id = rec_result.obj_id
            same_id_rec_result = left_arm_rec_result_list.get_snack_from_id(snack_id)
            if same_id_rec_result is not None:
                # 如果两个列表中都有，根据y值决定使用哪个结果
                if same_id_rec_result.base_coords[1] < 0 and rec_result.base_coords[1] < 0:
                    final_result.rec_result_list.append(rec_result)
                elif same_id_rec_result.base_coords[1] > 0 and rec_result.base_coords[1] > 0:
                    final_result.rec_result_list.append(same_id_rec_result)
                else:
                    raise ValueError("Rec result list fuse error G!")
                processed_ids.add(snack_id)
            else:
                # 只在self.rec_result_list中有
                final_result.rec_result_list.append(rec_result)

        # 处理left_arm_rec_result_list中独有的结果
        for rec_result in left_arm_rec_result_list.rec_result_list:
            if rec_result.obj_id not in processed_ids:
                final_result.rec_result_list.append(rec_result)

        return final_result
    
    # 过滤掉不需要的零食
    def filter(self,snack_ids:List[int])->Rec_result_list:
        final_result = Rec_result_list()
        for i in range(len(self.rec_result_list)):
            if self.rec_result_list[i].obj_id in snack_ids:
                final_result.rec_result_list.append(self.rec_result_list[i])
        return final_result
    
    def get_snack_from_id(self,snack_id:int)->Rec_result:
        for rec_result in self.rec_result_list:
            if rec_result.obj_id == snack_id:
                return rec_result
        return None

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
    

class Recognition_node():
    def __init__(self) -> None:
        # 订阅图像识别需求
        self.sub_request = rospy.Subscriber(utilis.Topic_name.image_recognition_request,msg.ImageRecRequest,self.do_image_rec_request,callback_args=self,queue_size=10)
        # 发布图像识别结果
        self.pub_result  = rospy.Publisher(utilis.Topic_name.image_recognition_result,msg.ImageRecResult,queue_size=10)
        # 请求手臂位置服务
        self.left_arm_client  = rospy.ServiceProxy(utilis.Topic_name.CheckLeftArmPose  ,srv.CheckArmPose)
        self.right_arm_client = rospy.ServiceProxy(utilis.Topic_name.CheckRightArmPose ,srv.CheckArmPose)
    
    @staticmethod
    # 图像识别请求回调
    def do_image_rec_request(request:msg.ImageRecRequest,self:Recognition_node):
        # rospy.loginfo(f"node name :{rospy.get_name()}, get request {request}")
        
        result = msg.ImageRecResult()
        # 识别零食,左右都要用
        if  request.task_type == task.Task_type.Task_image_rec.SNACK :
            snacks = request.snacks
            
            right_grabbed, right_img = right_camera.read()
            left_grabbed,  left_img  = left_camera.read()
            if right_grabbed == True and left_grabbed == True:
                timestamp = str(int(time.time()))
                cv2.imwrite(f"snack_right_{timestamp}.jpg", right_img)
                cv2.imwrite(f"snack_left_{timestamp}.jpg", left_img)
                timestamp = str(int(time.time()))
                timestamp = str(int(time.time()))
                right_stag_result = STag_rec(right_img,mtx, distCoeffs, utilis.Device_id.RIGHT, image_name=f"snack_right_{timestamp}")
                left_stag_result  = STag_rec(left_img, mtx, distCoeffs, utilis.Device_id.LEFT, image_name=f"snack_left_{timestamp}")
                
                # 请求机械臂位置
                arm_req = srv.CheckArmPoseRequest()
                arm_req.type_id = arm.PoseType.BASE_COORDS.value
                
                left_resp  = self.left_arm_client.call(arm_req)
                right_resp = self.right_arm_client.call(arm_req)
                
                right_arm_poses = right_resp.arm_pose
                right_stag_result.modified_position(request.task_type,utilis.Device_id.RIGHT,right_arm_poses)
                left_arm_poses  = left_resp.arm_pose
                left_stag_result.modified_position(request.task_type,utilis.Device_id.LEFT,left_arm_poses)
                
                # # YOLO识别
                # right_yolo_result = YOLO_rec(snacks, right_img)
                # left_yolo_result  = YOLO_rec(snacks, left_img)
                
                # # YOLO 与 STag 结果绑定
                # right_rec_result = right_stag_result.bind(right_yolo_result)
                # left_rec_result  = left_stag_result.bind(left_yolo_result)
                
                # STag 使用已知信息转为rec_result
                right_rec_result = right_stag_result.to_rec_result()
                left_rec_result  = left_stag_result.to_rec_result()
                
                # rospy.loginfo(f"right_rec_result : {right_rec_result}")
                # rospy.loginfo(f"left_rec_result : {left_rec_result}")
                
                # 两个结果融合
                fuse_rec_result = right_rec_result.fuse(left_rec_result)
                
                # 结果过滤
                final_rec_result = fuse_rec_result.filter(request.snacks)
                # rospy.loginfo(f"fuse_rec_result :\n {fuse_rec_result}")
                # rospy.loginfo(f"final_rec_result :\n {final_rec_result}")
                # 转为消息
                obj_positions = final_rec_result.to_msg()
                rospy.loginfo(f"snack rec finish")
            else:
                raise ValueError(f"get image False. Right Image: {right_grabbed}, Left Image: {left_grabbed}")
            
            
        # 识别杯子和咖啡机位置, 只有右臂
        elif request.task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE :
            # 拍摄图片
            grabbed, img = right_camera.read()
            if grabbed:
                timestamp = str(int(time.time()))
                firename = f'{LOG_DIR}/image/cup_coffee_{timestamp}.jpg'
                cv2.imwrite(firename, img)
                # STag 识别
                stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"cup_coffee_{timestamp}")
                
                # # 请求机械臂位置
                arm_req = srv.CheckArmPoseRequest()
                arm_req.type_id = arm.PoseType.BASE_COORDS.value
            
                right_resp = self.right_arm_client.call(arm_req)
                right_arm_poses = right_resp.arm_pose
                
                # 修正位置
                stag_result.modified_position(request.task_type,utilis.Device_id.RIGHT,right_arm_poses)
                
                # 发送信息
                obj_positions = stag_result.to_msg()
            else:
                    raise ValueError("get image False")

            
        # 识别咖啡机开关, 只有左臂
        elif request.task_type == task.Task_type.Task_image_rec.COFFEE_MACHINE_SWITCH_ON :
            # 拍摄图片
            grabbed, img = right_camera.read()
            if grabbed:
                timestamp = str(int(time.time()))
                firename = f'{LOG_DIR}/image/switch_on_{timestamp}.jpg'
                cv2.imwrite(firename, img)
                # STag识别
                stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"switch_on_{timestamp}")
                # # 请求机械臂位置
                arm_req = srv.CheckArmPoseRequest()
                arm_req.type_id = arm.PoseType.BASE_COORDS.value
            
                left_resp = self.left_arm_client.call(arm_req)
                left_arm_poses = left_resp.arm_pose
                
                # 修正位置
                stag_result.modified_position(request.task_type,utilis.Device_id.LEFT,left_arm_poses)
                
                # 发送信息
                obj_positions = stag_result.to_msg()
            else:
                    raise ValueError("get image False")
                
        # 识别咖啡机开关, 只有左臂
        elif request.task_type == task.Task_type.Task_image_rec.COFFEE_MACHINE_SWITCH_OFF :
            # 拍摄图片
            grabbed, img = right_camera.read()
            if grabbed:
                timestamp = str(int(time.time()))
                firename = f'{LOG_DIR}/image/switch_off_{timestamp}.jpg'
                cv2.imwrite(firename, img)
                # STag识别
                stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"switch_off_{timestamp}")
                # # 请求机械臂位置
                arm_req = srv.CheckArmPoseRequest()
                arm_req.type_id = arm.PoseType.BASE_COORDS.value
            
                left_resp = self.left_arm_client.call(arm_req)
                left_arm_poses = left_resp.arm_pose
                
                # 修正位置
                stag_result.modified_position(request.task_type,utilis.Device_id.LEFT,left_arm_poses)
                
                # 发送信息
                obj_positions = stag_result.to_msg()
            else:
                    raise ValueError("get image False")
        
        # 识别容器
        elif request.task_type == task.Task_type.Task_image_rec.CONTAINER:
            rospy.loginfo(f"request.task_type is task.Task_type.Task_image_rec.CONTAINER ")
            # 左臂
            if request.camera_id == utilis.Device_id.LEFT:
                # 拍摄图片
                grabbed, img = left_camera.read()
                if grabbed:
                    timestamp = str(int(time.time()))
                    firename = f'{LOG_DIR}/image/container_left_{timestamp}.jpg'
                    cv2.imwrite(firename, img)
                    rospy.loginfo(f"sava image {firename}")
                    stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"container_left_{timestamp}")
                    
                    # 请求机械臂位置
                    arm_req = srv.CheckArmPoseRequest()
                    arm_req.type_id = arm.PoseType.BASE_COORDS.value
                
                    left_resp:srv.CheckArmPoseResponse = self.left_arm_client.call(arm_req)
                    left_arm_poses = left_resp.arm_pose
                    
                    # 修正位置
                    stag_result.modified_position(request.task_type,utilis.Device_id.LEFT,left_arm_poses)
                    
                    # 发送信息
                    obj_positions = stag_result.to_msg()
                else:
                    raise ValueError("get image False")
                    
            elif request.camera_id == utilis.Device_id.RIGHT:
                # 拍摄图片
                grabbed, img = left_camera.read()
                if grabbed:
                    timestamp = str(int(time.time()))
                    firename = f'{LOG_DIR}/image/container_right_{timestamp}.jpg'
                    cv2.imwrite(firename, img)
                    stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"container_right_{timestamp}")
                    
                    # 请求机械臂位置
                    arm_req = srv.CheckArmPoseRequest()
                    arm_req.type_id = arm.PoseType.BASE_COORDS.value
                
                    right_resp      = self.right_arm_client.call(arm_req)
                    right_arm_poses = right_resp.arm_pose
                    
                    # 修正位置
                    stag_result.modified_position(request.task_type,utilis.Device_id.RIGHT,right_arm_poses)
                    
                    # 发送信息
                    obj_positions = stag_result.to_msg()
                else:
                    raise ValueError("get image False")
            else:
                raise ValueError("camera_id is not defined")
        else:
            raise ValueError("task_type is not defined")

        result.task_index    = request.task_index
        result.task_type     = request.task_type
        result.camera_id     = request.camera_id
        result.obj_positions = obj_positions
        # 发布结果
        rospy.loginfo(f"rec result :\n {result}")
        self.pub_result.publish(result)

# YOLO识别
def YOLO_rec(snack_id_list,image) -> YOLO_result_list:
    pass


# STag_rec识别
def STag_rec(image,mtx,distCoeffs,device_id:utilis.Device_id=utilis.Device_id.LEFT ,libraryHD=11,tag_size=24,image_name="")->STag_result_list:
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

    if image_name == "":
        image_name = int(time.time())

    cv2.imwrite(f'{LOG_DIR}/STag_result/STag_{image_name}.jpg', image)
    
    stag_result_list =  STag_result_list()
    
    # 对于每个id都要进行位置检测
    with open(f'{LOG_DIR}/STag_result/STag_{image_name}.txt', 'a') as file:
        for i, id in enumerate(ids):
            rospy.loginfo(f"Index: {i}, ID: {id[0]}")
            file.write(f"Index: {i}, ID: {id[0]}\n")
            imagePoints  = corners_list[i]
            success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs)
            if success:
                stag_result = STag_result(device_id, id[0], (imagePoints[0][0] + imagePoints[0][2])/2, [translationVector[0][0],translationVector[1][0],translationVector[2][0]])
                stag_result_list.add(stag_result)
                file.write(f"平移向量 x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}\n\n\n")
            else:
                print(f"{i} {id} Failed to solve PnP")
    
    return stag_result_list

# 偏移量
# TODO:后续用手眼矩阵覆盖一下
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

def init_camera_calibration():
    global mtx,distCoeffs
    # 内参矩阵
    # mtx =  [[1.06924343e+03, 0.00000000e+00, 6.87783503e+02],
    #         [0.00000000e+00, 1.06903179e+03, 4.49891603e+02],
    #         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
    mtx = np.array([[1.06924343e+03, 0.00000000e+00, 6.87783503e+02],
                    [0.00000000e+00, 1.06903179e+03, 4.49891603e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)
    # 畸变系数
    distCoeffs = np.array([1.836048608614118116e-01, -7.321056681877724515e-01, 
                           3.828147183491327119e-05, 1.508161050588582202e-03, 
                           9.120081467574957523e-01], dtype=np.float32)

def talker():
    # 初始化节点，命名为'camera'
    rospy.init_node('camera')
    
    init_camera()
    init_const()
    init_camera_calibration()

    recognition_node = Recognition_node()

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("camera")
        # 按照设定的频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
