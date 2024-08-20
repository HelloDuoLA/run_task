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
import std_srvs.srv as std_srvs

rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import utilis
import run_task.msg as msg
import run_task.srv as srv
import task
import order
import arm
import log 

# 摄像头识别节点, 完成摄像头的识别功能

# STag 识别结果
class STag_result():
    def __init__(self,camera_id:utilis.Device_id, stag_id:int, image_xy=[], image_coords=[]) -> None:
        self.camera_id    = camera_id
        self.stag_id      = stag_id
        self.image_xy     = image_xy
        self.image_coords = image_coords                 # 图像三维坐标系
        self.base_coords  = copy.deepcopy(image_coords)  # 机械臂基坐标系
        self.obj_id       = -999                         # 识别非零食使用

# STag 识别结果列表
class STag_result_list():
    # TODO:值域为STag码, 根据实际修改
    # STag -> snack ID
    STag_Snack_dict = {
        # 0 : order.Snack.Snack_id.CKU.value,
        # 0 : order.Snack.Snack_id.GUOSHU.value,
        0 : order.Snack.Snack_id.GUODONG.value,
        1 : order.Snack.Snack_id.RUSUANJUN.value,
        2 : order.Snack.Snack_id.CHENPIDAN.value,
        3 : order.Snack.Snack_id.YIDA.value,
    }
    
    STag_other_dict = {
        4 : task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT.value,
        5 : task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT.value,
        6 : task.Task_image_rec.Rec_OBJ_type.CUP.value,
        7 : task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_ON.value,
        8 : task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value
    }
    
    STag_other_enum_2_stag_num = {
        task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT    : 4,
        task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT   : 5,
        task.Task_image_rec.Rec_OBJ_type.CUP               : 6,
        task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_ON : 7,
        task.Task_image_rec.Rec_OBJ_type.WATER_POINT       : 8
    }
    
    
    def __init__(self) -> None:
        self.stag_result_list:List[STag_result] = []
    
    def add(self,stag_result:STag_result):
        self.stag_result_list.append(stag_result)   
    
    # 转为msg
    # def to_msg(self,) -> List[msg.ObjPositionWithID]:
    #     return_list = []
    #     for stag_result in self.stag_result_list:
    #         obj_position = msg.ObjPositionWithID()
    #         obj_position.arm_id        = stag_result.camera_id.value
    #         obj_position.obj_id        = stag_result.obj_id
    #         obj_position.position      = stag_result.base_coords
    #         obj_position.position_type = arm.PoseType.BASE_COORDS.value
    #         return_list.append(obj_position)
    #     return return_list
    

    # 通过已知STag id 与零食的关系进行绑定 
    def to_rec_result(self)->Rec_result_list:
        rec_result_list = Rec_result_list()
        for stag_result in self.stag_result_list:
            rec_result = Rec_result(stag_result.camera_id, self.STag_Snack_dict[stag_result.stag_id], stag_result.image_xy, stag_result.base_coords)
            rec_result_list.rec_result_list.append(rec_result)
        return rec_result_list
    
    # 根据任务与左右手对坐标值进行修正
    def modified_position(self,rec_task_type:task.Task_type.Task_image_rec,arm_id:utilis.Device_id,arm_poses):
        global  ev_left_arm_grip_container, ev_left_arm_grip_snack_top, ev_left_arm_grip_snack_bottom
        global  ev_left_arm_turn_off_machine, ev_left_arm_turn_on_machine, ev_right_arm_grip_container
        global  ev_right_arm_grip_cup, ev_right_arm_grip_snack_top, ev_right_arm_grip_snack_bottom, ev_right_arm_water_cup
        # 零食
        if rec_task_type == task.Task_type.Task_image_rec.SNACK:
            # 左臂
            if arm_id == utilis.Device_id.LEFT:
                for i in range(len(self.stag_result_list)):
                    stag_result = self.stag_result_list[i]
                    stag_result.base_coords[0] = arm_poses[0]  +  stag_result.image_coords[2] + LeftArmGripSnack.x    # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1]  -  stag_result.image_coords[1] + LeftArmGripSnack.y    # y = y - y + bias
                    stag_result.base_coords[2] = arm_poses[2]  +  stag_result.image_coords[0] + LeftArmGripSnack.z    # z = z + x + bias
                    
                    # 上层零食
                    if stag_result.base_coords[2] > 350 :
                        stag_result.base_coords[2] = LeftArmTopSnackGrip
                    # 下层零食
                    else:
                        stag_result.base_coords[2] = LeftArmBottomSnackGrip
                    # 记录经验值
                    log.log_empirical_value_left_arm_grip_snack(stag_result.base_coords)
            # 右臂
            elif arm_id == utilis.Device_id.RIGHT:
                for i in range(len(self.stag_result_list)):
                    stag_result = self.stag_result_list[i]
                    stag_result.base_coords[0] = arm_poses[0]  +  stag_result.image_coords[2] + RightArmGripSnack.x   # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1]  +  stag_result.image_coords[1] + RightArmGripSnack.y   # y = y + y + bias
                    stag_result.base_coords[2] = arm_poses[2]  -  stag_result.image_coords[0] + RightArmGripSnack.z   # z = z - x + bias
                    
                    # 上层零食
                    if stag_result.base_coords[2] > 350 :
                        stag_result.base_coords[2] = RightArmTopSnackGrip
                    # 下层零食
                    else:
                        stag_result.base_coords[2] = RightArmBottomSnackGrip
                        
                    # 记录经验值
                    log.log_empirical_value_right_arm_grip_snack(stag_result.base_coords)
        # 容器
        elif rec_task_type == task.Task_type.Task_image_rec.CONTAINER:
            if arm_id == utilis.Device_id.LEFT:
                # !左手
                new_stag_result_list = []
                for i in range(len(self.stag_result_list)):
                    stag_result = copy.deepcopy(self.stag_result_list[i]) 
                    # 寻找容器STag
                    if stag_result.stag_id == self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT]:
                        stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[0] + LeftArmGripContainer.x # x = x + x + bias
                        stag_result.base_coords[1] = arm_poses[1] - stag_result.image_coords[1] + LeftArmGripContainer.y # y = y - y + bias
                        stag_result.base_coords[2] = LeftArmGripContainer.const_z                     #固定z坐标
                        stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT.value
                        new_stag_result_list.append(stag_result)
                        # 记录经验值
                        log.log_empirical_value_left_arm_grip_container(stag_result.base_coords)
                        
                        # 零食放置点
                        put_snack_point = copy.deepcopy(stag_result)
                        put_snack_point.base_coords[0] = stag_result.base_coords[0] + LeftArmLossenSnack.x
                        put_snack_point.base_coords[1] = stag_result.base_coords[1] + LeftArmLossenSnack.y
                        put_snack_point.base_coords[2] = stag_result.base_coords[2] + LeftArmLossenSnack.z
                        put_snack_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.LOSSEN_SNACK.value     
                        new_stag_result_list.append(put_snack_point)
                        
                        # 手臂躲避点
                        arm_dodge_point = copy.deepcopy(stag_result)
                        arm_dodge_point.base_coords[0] = stag_result.base_coords[0] + LeftArmGripContainerDodge.x
                        arm_dodge_point.base_coords[1] = stag_result.base_coords[1] + LeftArmGripContainerDodge.y
                        arm_dodge_point.base_coords[2] = stag_result.base_coords[2] + LeftArmGripContainerDodge.z
                        arm_dodge_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.CONTAINER_DODGE.value
                        new_stag_result_list.append(arm_dodge_point)
                        
                
                # !使用经验值(没有识别到容器框)
                if len(new_stag_result_list) == 0:
                    rospy.loginfo("No container STag detected!!!!!!! use experience value")
                    stag_result          = STag_result(utilis.Device_id.LEFT, self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT])
                    stag_result.stag_id  =  self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT]
                    stag_result.base_coords = ev_left_arm_grip_container                  
                    stag_result.obj_id    = task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT.value
                    new_stag_result_list.append(stag_result)
                    
                    # 零食放置点
                    put_snack_point = copy.deepcopy(stag_result)
                    put_snack_point.base_coords[0] = stag_result.base_coords[0] + LeftArmLossenSnack.x
                    put_snack_point.base_coords[1] = stag_result.base_coords[1] + LeftArmLossenSnack.y
                    put_snack_point.base_coords[2] = stag_result.base_coords[2] + LeftArmLossenSnack.z
                    put_snack_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.LOSSEN_SNACK.value
                    new_stag_result_list.append(put_snack_point)
                    
                    # 躲闪点
                    dodge_point = copy.deepcopy(stag_result)
                    dodge_point.base_coords[0] = stag_result.base_coords[0] + LeftArmGripContainerDodge.x
                    dodge_point.base_coords[1] = stag_result.base_coords[1] + LeftArmGripContainerDodge.y
                    dodge_point.base_coords[2] = stag_result.base_coords[2] + LeftArmGripContainerDodge.z
                    dodge_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.CONTAINER_DODGE.value
                    new_stag_result_list.append(dodge_point)
                        
                self.stag_result_list = new_stag_result_list
            elif arm_id == utilis.Device_id.RIGHT:
                # !右手
                new_stag_result_list = [ ]
                for i in range(len(self.stag_result_list)):
                    stag_result = copy.deepcopy(self.stag_result_list[i]) 
                    # 寻找容器STag
                    if stag_result.stag_id == self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT]:
                        # 抓容器准备位点
                        stag_result.base_coords[0] = arm_poses[0] - stag_result.image_coords[0] + RightArmGripContainer.x # x = x - x + bias
                        stag_result.base_coords[1] = arm_poses[1] + stag_result.image_coords[1] + RightArmGripContainer.y # y = y + y + bias
                        stag_result.base_coords[2] = RightArmGripContainer.const_z                     #固定z坐标
                        stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT.value
                        new_stag_result_list.append(stag_result)
                        # 记录经验值
                        log.log_empirical_value_right_arm_grip_container(stag_result.base_coords)
                        
                        # 零食放置点
                        put_snack_point = copy.deepcopy(stag_result)
                        put_snack_point.base_coords[0] = stag_result.base_coords[0] + RightArmLossenSnack.x
                        put_snack_point.base_coords[1] = stag_result.base_coords[1] + RightArmLossenSnack.y
                        put_snack_point.base_coords[2] = stag_result.base_coords[2] + RightArmLossenSnack.z
                        put_snack_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.LOSSEN_SNACK.value
                        new_stag_result_list.append(put_snack_point)
                        
                        # 躲闪点
                        dodge_point = copy.deepcopy(stag_result)
                        dodge_point.base_coords[0] = stag_result.base_coords[0] + RightArmGripContainerDodge.x
                        dodge_point.base_coords[1] = stag_result.base_coords[1] + RightArmGripContainerDodge.y
                        dodge_point.base_coords[2] = stag_result.base_coords[2] + RightArmGripContainerDodge.z
                        dodge_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.CONTAINER_DODGE.value
                        new_stag_result_list.append(dodge_point)
                        
                
                # 使用经验值(没有识别到容器框)
                if len(new_stag_result_list) == 0:
                    rospy.loginfo("No container STag detected!!!!!!!! use experience value")
                    stag_result             = STag_result(utilis.Device_id.RIGHT, self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT])
                    stag_result.stag_id     = self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT]
                    stag_result.base_coords = ev_right_arm_grip_container                  
                    stag_result.obj_id      = task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT.value
                    new_stag_result_list.append(stag_result)
                    
                    # 零食放置点
                    put_snack_point = copy.deepcopy(stag_result)
                    put_snack_point.base_coords[0] = stag_result.base_coords[0] + RightArmLossenSnack.x
                    put_snack_point.base_coords[1] = stag_result.base_coords[1] + RightArmLossenSnack.y
                    put_snack_point.base_coords[2] = stag_result.base_coords[2] + RightArmLossenSnack.z
                    put_snack_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.LOSSEN_SNACK.value
                    new_stag_result_list.append(put_snack_point)
                    
                    # 躲闪点
                    dodge_point = copy.deepcopy(stag_result)
                    dodge_point.base_coords[0] = stag_result.base_coords[0] + RightArmGripContainerDodge.x
                    dodge_point.base_coords[1] = stag_result.base_coords[1] + RightArmGripContainerDodge.y
                    dodge_point.base_coords[2] = stag_result.base_coords[2] + RightArmGripContainerDodge.z
                    dodge_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.CONTAINER_DODGE.value
                    new_stag_result_list.append(dodge_point)
                        
                self.stag_result_list = new_stag_result_list
        
        # 咖啡机和杯子
        elif rec_task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE:
            new_stag_result_list = [ ]
            # 只有右手
            for i in range(len(self.stag_result_list)):
                stag_result = copy.deepcopy(self.stag_result_list[i])  
                # 先判断是不是杯子或接水点
                # !修改为只识别杯子
                if stag_result.stag_id == self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CUP]:
                    # 杯子
                    stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[2] + RightArmGripCup.x  # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1] + stag_result.image_coords[1] + RightArmGripCup.y  # y = y + y + bias
                    stag_result.base_coords[2] = RightArmGripCup.const_z                              #固定z坐标
                    stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.CUP.value
                    new_stag_result_list.append(stag_result)
                    # 记录经验值
                    log.log_empirical_value_right_arm_grip_cup(stag_result.base_coords)
                # elif stag_result.stag_id == self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.WATER_POINT]:
                #     # 接水点
                #     stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[2] + RightArmWaterCup.x  # x = x + z + bias
                #     stag_result.base_coords[1] = arm_poses[1] + stag_result.image_coords[1] + RightArmWaterCup.y  # y = y + y + bias
                #     stag_result.base_coords[2] = RightArmWaterCup.const_z                             #固定z坐标
                #     stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value
                #     new_stag_result_list.append(stag_result)
                #     # 记录经验值
                #     log.log_empirical_value_right_arm_water_cup(stag_result.base_coords)
            
            # !加入经验值
            if len(new_stag_result_list) != 1:
                rospy.loginfo("No cup or water point STag detected")
                
                if len(new_stag_result_list) == 1:
                    # 接水点没有识别到
                    if new_stag_result_list[0].obj_id == task.Task_image_rec.Rec_OBJ_type.CUP.value:
                        # rospy.loginfo("No water point STag detected !!!!!!! use experience value")
                        # water_point = STag_result(utilis.Device_id.RIGHT,self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.WATER_POINT])
                        # water_point.base_coords = ev_right_arm_water_cup
                        # water_point.obj_id = task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value
                        # new_stag_result_list.append(water_point)
                        pass
                    # 杯子没有识别到
                    elif new_stag_result_list[0].obj_id == task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value:
                        rospy.loginfo("No cup STag detected !!!!!!! use experience value")
                        grip_cup_point = STag_result(utilis.Device_id.RIGHT,self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CUP])
                        grip_cup_point.base_coords = ev_right_arm_grip_cup
                        grip_cup_point.obj_id = task.Task_image_rec.Rec_OBJ_type.CUP.value
                        new_stag_result_list.append(grip_cup_point)
                
                # 接水点和杯子都没有识别到
                elif len(new_stag_result_list) == 0:
                    rospy.loginfo("No cup or water point STag detected !!!!!!! use experience value")
                    grip_cup_point = STag_result(utilis.Device_id.RIGHT,self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.CUP])
                    grip_cup_point.base_coords = ev_right_arm_grip_cup
                    grip_cup_point.obj_id = task.Task_image_rec.Rec_OBJ_type.CUP.value
                    new_stag_result_list.append(grip_cup_point)
                    
                    # water_point = STag_result(utilis.Device_id.RIGHT,self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.WATER_POINT])
                    # water_point.base_coords = ev_right_arm_water_cup
                    # water_point.obj_id = task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value
                    # new_stag_result_list.append(water_point)
            self.stag_result_list = new_stag_result_list
            
        # 打开咖啡机开关
        elif rec_task_type == task.Task_type.Task_image_rec.COFFEE_MACHINE_SWITCH_ON:
            # 识别开机, 只有左手
            new_stag_result_list = [ ]
            for i in range(len(self.stag_result_list)):
                stag_result = copy.deepcopy(self.stag_result_list[i])  
                if stag_result.stag_id == self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_ON]:
                    # 开机键
                    stag_result.base_coords[0] = arm_poses[0] + stag_result.image_coords[2] + LeftArmGripTurnOnMachineSwitch.x  # x = x + z + bias
                    stag_result.base_coords[1] = arm_poses[1] - stag_result.image_coords[1] + LeftArmGripTurnOnMachineSwitch.y  # y = y - y + bias
                    stag_result.base_coords[2] = arm_poses[2] + stag_result.image_coords[0] + LeftArmGripTurnOnMachineSwitch.z  # z = z + x + bias
                    stag_result.obj_id = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_ON.value
                    new_stag_result_list.append(stag_result)
                    # 记录经验值
                    log.log_empirical_value_left_arm_turn_on_machine(stag_result.base_coords)
                    
                    # 关机键
                    turn_off_point = copy.deepcopy(stag_result)
                    turn_off_point.base_coords[0] = stag_result.base_coords[0] + LeftArmGripTurnOFFMachineSwitch.x
                    turn_off_point.base_coords[1] = stag_result.base_coords[1] + LeftArmGripTurnOFFMachineSwitch.y
                    turn_off_point.base_coords[2] = stag_result.base_coords[2] + LeftArmGripTurnOFFMachineSwitch.z
                    turn_off_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_OFF.value
                    new_stag_result_list.append(turn_off_point)
                    
                    # 转移点
                    tran_point = copy.deepcopy(stag_result)
                    tran_point.base_coords[0] = stag_result.base_coords[0] + LeftArmGripTranMachineSwitch.x
                    tran_point.base_coords[1] = stag_result.base_coords[1] + LeftArmGripTranMachineSwitch.y
                    tran_point.base_coords[2] = stag_result.base_coords[2] + LeftArmGripTranMachineSwitch.z
                    tran_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_TRA.value
                    new_stag_result_list.append(tran_point)
                    
                    # 接水点
                    water_point = copy.deepcopy(stag_result)
                    water_point.base_coords[0] = stag_result.base_coords[0] + LeftArmWaterCup.x
                    water_point.base_coords[1] = stag_result.base_coords[1] + LeftArmWaterCup.y
                    water_point.base_coords[2] = LeftArmWaterCup.const_z                            # 固定值
                    water_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value
                    new_stag_result_list.append(water_point)
                    
            
            # 加入经验值
            # TODO: 待修改
            if len(new_stag_result_list) == 0:
                rospy.loginfo("No machine switch STag detected !!!!!!! use experience value")
                stag_result = STag_result(utilis.Device_id.LEFT,self.STag_other_enum_2_stag_num[task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH])
                stag_result.base_coords = ev_left_arm_turn_on_machine
                stag_result.obj_id      = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_ON.value
                new_stag_result_list.append(stag_result)
                
                # 关机键
                turn_off_point = copy.deepcopy(stag_result)
                turn_off_point.base_coords[0] = stag_result.base_coords[0] + LeftArmGripTurnOFFMachineSwitch.x
                turn_off_point.base_coords[1] = stag_result.base_coords[1] + LeftArmGripTurnOFFMachineSwitch.y
                turn_off_point.base_coords[2] = stag_result.base_coords[2] + LeftArmGripTurnOFFMachineSwitch.z
                turn_off_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_OFF.value
                new_stag_result_list.append(turn_off_point)
                
                # 转移点
                tran_point = copy.deepcopy(stag_result)
                tran_point.base_coords[0] = stag_result.base_coords[0] + LeftArmGripTranMachineSwitch.x
                tran_point.base_coords[1] = stag_result.base_coords[1] + LeftArmGripTranMachineSwitch.y
                tran_point.base_coords[2] = stag_result.base_coords[2] + LeftArmGripTranMachineSwitch.z
                tran_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_TRA.value
                new_stag_result_list.append(tran_point)
                
                # 接水点
                water_point = copy.deepcopy(stag_result)
                water_point.base_coords[0] = stag_result.base_coords[0] + LeftArmWaterCup.x
                water_point.base_coords[1] = stag_result.base_coords[1] + LeftArmWaterCup.y
                water_point.base_coords[2] = LeftArmWaterCup.const_z                            # 固定值
                water_point.obj_id  = task.Task_image_rec.Rec_OBJ_type.WATER_POINT.value
                new_stag_result_list.append(water_point)
                
            
            self.stag_result_list = new_stag_result_list
            
        else:
            raise ValueError("rec_task_type is not defined")

# 识别结果
class Rec_result():
    def __init__(self,camera_id:utilis.Device_id, obj_id:order.Snack.Snack_id, image_xy=[], base_coords=[]) -> None:
        self.camera_id   = camera_id     # 摄像头ID
        self.obj_id      = obj_id        # 物体ID
        self.image_xy    = image_xy      # 图像坐标
        self.base_coords = base_coords   # 机械臂基坐标系
    
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
            obj_position.arm_id        = stag_result.camera_id.value
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
    def __init__(self,snack_tag, bonding_box = [], confidence = 0) -> None:
        self.snack_tag           = snack_tag             # 零食标签 
        self.bonding_box         = bonding_box           # 检测框
        self.confidence          = confidence            # 置信度
    


# YOLO 识别结果列表
class YOLO_result_list():
    def __init__(self) -> None:
        self.yolo_result_list:List[YOLO_result] = []
    
    # 增加
    def add(self,yolo_result:YOLO_result):
        self.yolo_result_list.append(yolo_result)
    
    # TODO:在图像上画结果
    def draw_result(image,image_name=""):
        pass
    
    # 转为目标识别结果
    def to_obj_result(self, camera_id:utilis.Device_id)->Obj_result_list:
        obj_result_list = Obj_result_list()
        for yolo_result in self.yolo_result_list:
            obj_result = Obj_result(camera_id,yolo_result.snack_tag, yolo_result.bonding_box)
            obj_result_list.add(obj_result)

# OBJ 识别结果
class Obj_result():
    # Tag-> ID
    Tag_Snack_dict = {
        "guodong"   : order.Snack.Snack_id.GUODONG.value,
        "yiliduo"   : order.Snack.Snack_id.RUSUANJUN.value,
        "chenpidan" : order.Snack.Snack_id.CHENPIDAN.value,
        "yida"      : order.Snack.Snack_id.YIDA.value,
    }
    
    def __init__(self,camera_id:utilis.Device_id,snack_tag,bonding_box) -> None:
        self.camera_id    = camera_id                            # 摄像头id
        self.snack_tag    = snack_tag                            # 零食标签 
        self.obj_id       = self.Tag_Snack_dict[snack_tag]       # 零食id
        self.bonding_box  = bonding_box                          # bonding_box
        self.image_xy     = (bonding_box[0] + bonding_box[2])/2  # 检测框中心点
        self.image_coords = None                                 # 图像三维坐标系
        self.base_coords  = None                                 # 机械臂基坐标系

# OBJ 识别结果列表
class  Obj_result_list():
    # 物体真实宽度
    Obj_True_Width = {
        order.Snack.Snack_id.GUODONG.value   : 8 ,
        order.Snack.Snack_id.RUSUANJUN.value : 4.5 ,
        order.Snack.Snack_id.CHENPIDAN.value : 4.6 ,
        order.Snack.Snack_id.YIDA.value      : 5.6  ,
    }
    
    # 物体真实高度
    Obj_True_Height = {
        order.Snack.Snack_id.GUODONG.value   : 13.5 ,
        order.Snack.Snack_id.RUSUANJUN.value : 8.7 ,
        order.Snack.Snack_id.CHENPIDAN.value : 8.7 ,
        order.Snack.Snack_id.YIDA.value      : 8.3  ,
    }
    

    def __init__(self) -> None:
        self.obj_result_list:List[Obj_result] = []
    
    # 增加
    def add(self,obj_result:Obj_result):
        self.obj_result_list.append(obj_result)
    
    # 识别
    def rec(self,mtx,distCoeffs,image_name):
        for obj_result in self.obj_result_list:
            true_width = self.Obj_True_Width[obj_result.obj_id]
            true_height = self.Obj_True_Height[obj_result.obj_id]
            objectPoints = np.array([
                [-true_height/2, -true_width/2,  0],
                [true_width/2 , -true_width/2,  0],
                [true_width/2 , true_width/2 ,  0],
                [-true_width/2, true_width/2 ,  0]
            ], dtype=np.float32)   
            imagePoints  = obj_result.bonding_box       
            success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs)
            obj_result.image_coords = [translationVector[0][0],translationVector[1][0],translationVector[2][0]]
            
            log.log_stag_result(f'{image_name}_STag.txt',f"平移向量 x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}\n\n\n")
        
    # 根据任务与左右手对坐标值进行修正
    def modified_position(self,rec_task_type:task.Task_type.Task_image_rec,arm_id:utilis.Device_id,arm_poses):
        global  ev_left_arm_grip_container, ev_left_arm_grip_snack_top, ev_left_arm_grip_snack_bottom
        global  ev_left_arm_turn_off_machine, ev_left_arm_turn_on_machine, ev_right_arm_grip_container
        global  ev_right_arm_grip_cup, ev_right_arm_grip_snack_top, ev_right_arm_grip_snack_bottom, ev_right_arm_water_cup
        # 零食
        if rec_task_type == task.Task_type.Task_image_rec.SNACK:
            # 左臂
            if arm_id == utilis.Device_id.LEFT:
                for i in range(len(self.obj_result_list)):
                    obj_result = self.obj_result_list[i]
                    obj_result.base_coords[0] = arm_poses[0]  +  obj_result.image_coords[2] + LeftArmGripSnackDNN.x    # x = x + z + bias
                    obj_result.base_coords[1] = arm_poses[1]  -  obj_result.image_coords[1] + LeftArmGripSnackDNN.y    # y = y - y + bias
                    obj_result.base_coords[2] = arm_poses[2]  +  obj_result.image_coords[0] + LeftArmGripSnackDNN.z    # z = z + x + bias
                    
                    # 上层零食
                    if obj_result.base_coords[2] > 350 :
                        obj_result.base_coords[2] = LeftArmTopSnackGrip
                    # 下层零食
                    else:
                        obj_result.base_coords[2] = LeftArmBottomSnackGrip
                    # 记录经验值
                    log.log_empirical_value_left_arm_grip_snack(obj_result.base_coords)
            # 右臂
            elif arm_id == utilis.Device_id.RIGHT:
                for i in range(len(self.obj_result_list)):
                    obj_result = self.obj_result_list[i]
                    obj_result.base_coords[0] = arm_poses[0]  +  obj_result.image_coords[2] + RightArmGripSnackDNN.x   # x = x + z + bias
                    obj_result.base_coords[1] = arm_poses[1]  +  obj_result.image_coords[1] + RightArmGripSnackDNN.y   # y = y + y + bias
                    obj_result.base_coords[2] = arm_poses[2]  -  obj_result.image_coords[0] + RightArmGripSnackDNN.z   # z = z - x + bias
                    
                    # 上层零食
                    if obj_result.base_coords[2] > 350 :
                        obj_result.base_coords[2] = RightArmTopSnackGrip
                    # 下层零食
                    else:
                        obj_result.base_coords[2] = RightArmBottomSnackGrip
                        
                    # 记录经验值
                    log.log_empirical_value_right_arm_grip_snack(obj_result.base_coords)
        else:
            raise ValueError("rec_task_type is not defined")

    # 通过已知零食标签 与零食id的关系进行绑定, 转为rec_result 
    def to_rec_result(self)->Rec_result_list:
        rec_result_list = Rec_result_list()
        for obj_result in self.obj_result_list:
            rec_result = Rec_result(obj_result.camera_id, obj_result.obj_id, obj_result.image_xy, obj_result.base_coords)
            rec_result_list.rec_result_list.append(rec_result)
        return rec_result_list

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
        rospy.loginfo(f"node name :{rospy.get_name()}, get request {request}")
        result = msg.ImageRecResult()
        # 识别零食,左右都要用
        if  request.task_type == task.Task_type.Task_image_rec.SNACK :            
            right_grabbed, right_img = right_camera.read()
            left_grabbed,  left_img  = left_camera.read()
            if right_grabbed == True and left_grabbed == True:
                timestamp = str(int(time.time()))
                # 记录图片
                log.log_write_image(f"{timestamp}_snack_right.jpg", right_img)
                log.log_write_image(f"{timestamp}_snack_left.jpg", left_img)
                timestamp = str(int(time.time()))
                timestamp = str(int(time.time()))
                
                if request.rec_method == utilis.Rec_method.STAG:
                    # STag 识别零食
                    right_stag_result = STag_rec(right_img,mtx, distCoeffs, utilis.Device_id.RIGHT, image_name=f"{timestamp}_snack_right")
                    left_stag_result  = STag_rec(left_img, mtx, distCoeffs, utilis.Device_id.LEFT, image_name=f"{timestamp}_snack_left")
                    
                    # 请求机械臂位置
                    arm_req = srv.CheckArmPoseRequest()
                    arm_req.type_id = arm.PoseType.BASE_COORDS.value
                    
                    left_resp  = self.left_arm_client.call(arm_req)
                    right_resp = self.right_arm_client.call(arm_req)
                    
                    # 修正角度
                    right_arm_poses = right_resp.arm_pose
                    right_stag_result.modified_position(request.task_type,utilis.Device_id.RIGHT,right_arm_poses)
                    left_arm_poses  = left_resp.arm_pose
                    left_stag_result.modified_position(request.task_type,utilis.Device_id.LEFT,left_arm_poses)
                    
                    # STag 使用已知信息转为rec_result
                    right_rec_result = right_stag_result.to_rec_result()
                    left_rec_result  = left_stag_result.to_rec_result()
                    
                    # 两个结果融合
                    fuse_rec_result = right_rec_result.fuse(left_rec_result)
                    
                    # 结果过滤
                    final_rec_result = fuse_rec_result.filter(request.snacks)

                    # 转为消息
                    obj_positions = final_rec_result.to_msg()
                    rospy.loginfo(f"snack rec finish")
                elif request.rec_method == utilis.Rec_method.YOLO:
                    pass
            else:
                raise ValueError(f"get image False. Right Image: {right_grabbed}, Left Image: {left_grabbed} !!!!")

        # 识别容器
        elif request.task_type == task.Task_type.Task_image_rec.CONTAINER:
            rospy.loginfo(f"request.task_type is task.Task_type.Task_image_rec.CONTAINER ")
            # 左臂
            if request.camera_id == utilis.Device_id.LEFT:
                # 拍摄图片
                grabbed, img = left_camera.read()
                if grabbed:
                    timestamp = str(int(time.time()))
                    firename = f'{timestamp}_container_left.jpg'
                    log.log_write_image(firename, img)
                    stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"{timestamp}_container_left")
                    
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
                grabbed, img = right_camera.read()
                if grabbed:
                    timestamp = str(int(time.time()))
                    firename = f'{timestamp}_container_right.jpg'
                    log.log_write_image(firename, img)
                    stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"{timestamp}_container_right")
                    
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
        # 识别杯子和咖啡机位置, 只有右臂
        elif request.task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE :
            # 拍摄图片
            grabbed, img = right_camera.read()
            if grabbed:
                timestamp = str(int(time.time()))
                firename = f'{timestamp}_cup_coffee_.jpg'
                log.log_write_image(firename, img)
                
                # STag 识别
                stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"{timestamp}_cup_coffee_.jpg")
                
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
            grabbed, img = left_camera.read()
            if grabbed:
                timestamp = str(int(time.time()))
                firename = f'{timestamp}_switch_on.jpg'
                log.log_write_image(firename, img)
                # STag识别
                stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"{timestamp}_switch_on")
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
            grabbed, img = left_camera.read()
            if grabbed:
                timestamp = str(int(time.time()))
                firename = f'{timestamp}_switch_off.jpg'
                log.log_write_image(firename, img)
                # STag识别
                stag_result = STag_rec(img,mtx,distCoeffs,image_name=f"{timestamp}_switch_off")
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
        else:
            raise ValueError("task_type is not defined")

        result.task_index    = request.task_index
        result.task_type     = request.task_type
        result.camera_id     = request.camera_id
        result.obj_positions = obj_positions
        # 发布结果
        rospy.loginfo(f"rec result :\n {result}")
        self.pub_result.publish(result)
        # rospy.loginfo(f"rec send result :\n")

# YOLO检测函数
def YOLO_detect():
    # 顺时针4个点，左上角为起点
    obj1 = ["yiliduo",[[1,2],[3,4],[5,6],[7,8]],"填入置信度的值"]
    obj2 = ["guodong",[[1,2],[3,4],[5,6],[7,8]],"填入置信度的值"]
    obj_list = []
    obj_list.append(obj1)
    obj_list.append(obj2)
    return obj_list

# OBJ识别
def Obj_rec(image,mtx,distCoeffs,device_id:utilis.Device_id=utilis.Device_id.LEFT, image_name="") -> Obj_result_list:
    # 获取图像的尺寸
    height, width = image.shape[:2]

    # 计算图像中心作为圆心坐标
    center_coordinates = (width // 2, height // 2)
    radius = 5  # 圆的半径
    color = (0, 255, 0)  # BGR颜色，这里为绿色
    thickness = -1  # 设置为-1表示填充整个圆
    
    # 检测结果列表
    detect_result_list = YOLO_detect()
    
    a = Obj_result_list()
    return a


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

    log.log_write_image(f'STag_{image_name}.jpg', image)
    # 结果列表
    stag_result_list =  STag_result_list()
    
    # 对于每个id都要进行位置检测
    for i, id in enumerate(ids):
        rospy.loginfo(f"Index: {i}, ID: {id[0]}")
        log.log_stag_result(f'{image_name}_STag.txt',f"Index: {i}, ID: {id[0]}\n")
        imagePoints  = corners_list[i]
        # flags=cv2.SOLVEPNP_IPPE_SQUARE # 参数有毒
        # success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs,flags=cv2.SOLVEPNP_IPPE_SQUARE)
        success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs)
        if success:
            stag_result = STag_result(device_id, id[0], (imagePoints[0][0] + imagePoints[0][2])/2, [translationVector[0][0],translationVector[1][0],translationVector[2][0]])
            stag_result_list.add(stag_result)
            log.log_stag_result(f'{image_name}_STag.txt',f"平移向量 x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}\n\n\n")
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
    
    def __str__(self) -> str:
        return f"x : {self.x} y : {self.y} z : {self.z} const_z : {self.const_z}"

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
    
    # camera_ids = [4, 6]
    # cameras = {}

    # for cid in camera_ids:
    #     # 构建GStreamer管道字符串，包括帧率设置
    #     gstreamer_pipeline = (
    #         f'v4l2src device=/dev/video{cid} ! '
    #         'video/x-raw, format=(string)YUY2, width=(int)1280, height=(int)960, framerate=(fraction)5/1 ! '
    #         'queue max-size-buffers=1 ! '
    #         'videoconvert ! appsink'
    #     )
    #     cameras[cid] = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

    # # 检查摄像头是否正确打开
    # for cid, camera in cameras.items():
    #     if not camera.isOpened():
    #         print(f"摄像头 {cid} 无法打开。")
    #     else:
    #         print(f"摄像头 {cid} 已成功打开。")
            
    # left_camera  = cameras[4]
    # right_camera = cameras[6]
    
    # gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg, width=640, height=480, framerate=30/1 ! jpegdec ! videoconvert ! appsink sync=false emit-signals=true max-buffers=1 drop=true
    
    # left_camera_id  = 4
    # right_camera_id = 6
    
    left_camera_id  = 6
    right_camera_id = 4
        
        
    # left_camera     = cv2.VideoCapture(left_camera_id, cv2.CAP_V4L2)
    # right_camera    = cv2.VideoCapture(right_camera_id, cv2.CAP_V4L2)  
    
    # left_camera     = cv2.VideoCapture(left_camera_id, cv2.CAP_GSTREAMER)
    # right_camera    = cv2.VideoCapture(right_camera_id, cv2.CAP_GSTREAMER)
    
    left_camera     = cv2.VideoCapture(left_camera_id)
    right_camera    = cv2.VideoCapture(right_camera_id)
    
    
    # frame_width     = rospy.get_param(f'~frame_width',  1280)
    # frame_height    = rospy.get_param(f'~frame_height', 960)    
    frame_width     = rospy.get_param(f'~frame_width',  640)
    frame_height    = rospy.get_param(f'~frame_height', 480)
    
    left_camera.set(cv2.CAP_PROP_FRAME_WIDTH , frame_width)
    left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    
    right_camera.set(cv2.CAP_PROP_FRAME_WIDTH , frame_width)
    right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

# 初始化偏移常量
def init_const():
    global LeftArmGripSnack, LeftArmGripContainer, LeftArmGripTurnOnMachineSwitch, LeftArmGripTurnOFFMachineSwitch, LeftArmGripTranMachineSwitch
    global RightArmGripSnack, RightArmGripContainer, RightArmGripCup
    global LeftArmLossenSnack, RightArmLossenSnack 
    global LeftArmGripContainerDodge, RightArmGripContainerDodge
    global RightArmTopSnackGrip,RightArmBottomSnackGrip
    global LeftArmTopSnackGrip,LeftArmBottomSnackGrip
    
    global LeftArmWaterCup, RightArmWaterCup
    
    global LeftArmGripSnackDNN, RightArmGripSnackDNN
    
    # 获取项偏移
    LeftArmGripSnack                = get_deviation("LeftArmGripSnack")
    LeftArmGripSnackDNN             = get_deviation("LeftArmGripSnackDNN")
    LeftArmLossenSnack              = get_deviation("LeftArmLossenSnack")
    LeftArmGripContainer            = get_deviation("LeftArmGripContainer",True)
    LeftArmGripContainerDodge       = get_deviation("LeftArmGripContainerDodge",True)
    LeftArmGripTurnOnMachineSwitch  = get_deviation("LeftArmGripTurnOnMachineSwitch")
    LeftArmGripTranMachineSwitch    = get_deviation("LeftArmGripTranMachineSwitch")
    LeftArmGripTurnOFFMachineSwitch = get_deviation("LeftArmGripTurnOFFMachineSwitch")
    LeftArmWaterCup                 = get_deviation("LeftArmWaterCup",True)
    RightArmGripSnack               = get_deviation("RightArmGripSnack")
    RightArmGripSnackDNN            = get_deviation("RightArmGripSnackDNN")
    RightArmLossenSnack             = get_deviation("RightArmLossenSnack")
    RightArmGripContainer           = get_deviation("RightArmGripContainer",True)
    RightArmGripContainerDodge      = get_deviation("RightArmGripContainerDodge",True)
    RightArmGripCup                 = get_deviation("RightArmGripCup",True)
    RightArmWaterCup                = get_deviation("RightArmWaterCup",True)
    RightArmTopSnackGrip            = rospy.get_param(f'~RightArmTopSnackGrip/z_const')
    RightArmBottomSnackGrip         = rospy.get_param(f'~RightArmBottomSnackGrip/z_const')
    LeftArmTopSnackGrip             = rospy.get_param(f'~LeftArmTopSnackGrip/z_const')
    LeftArmBottomSnackGrip          = rospy.get_param(f'~LeftArmBottomSnackGrip/z_const')
    
def init_empirical_value():   
    global  ev_left_arm_grip_container, ev_left_arm_grip_snack_top, ev_left_arm_grip_snack_bottom
    global  ev_left_arm_turn_off_machine, ev_left_arm_turn_on_machine, ev_right_arm_grip_container
    global  ev_right_arm_grip_cup, ev_right_arm_grip_snack_top, ev_right_arm_grip_snack_bottom, ev_right_arm_water_cup
    
    ev_left_arm_grip_container    = _get_arm_empirical_value("left_arm_grip_container")
    ev_left_arm_grip_snack_top    = _get_arm_empirical_value("left_arm_grip_snack_top")
    ev_left_arm_grip_snack_bottom = _get_arm_empirical_value("left_arm_grip_snack_bottom")
    ev_left_arm_turn_off_machine  = _get_arm_empirical_value("left_arm_turn_off_machine")
    ev_left_arm_turn_on_machine   = _get_arm_empirical_value("left_arm_turn_on_machine")
    ev_right_arm_grip_container   = _get_arm_empirical_value("right_arm_grip_container")
    ev_right_arm_grip_cup         = _get_arm_empirical_value("right_arm_grip_cup")
    ev_right_arm_grip_snack_top   = _get_arm_empirical_value("right_arm_grip_snack_top")
    ev_right_arm_grip_snack_bottom= _get_arm_empirical_value("right_arm_grip_snack_bottom")
    ev_right_arm_water_cup        = _get_arm_empirical_value("right_arm_water_cup")
# 通过名字获取经验值
def _get_arm_empirical_value(anchor_point_name):
    xyz = rospy.get_param(f'~{anchor_point_name}/xyz')
    return xyz

def init_camera_calibration():
    global mtx,distCoeffs
    
    # 1280 * 960
    # 内参矩阵
    # mtx = np.array([[1.06924343e+03, 0.00000000e+00, 6.87783503e+02],
    #                 [0.00000000e+00, 1.06903179e+03, 4.49891603e+02],
    #                 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)
    # # 畸变系数
    # distCoeffs = np.array([1.836048608614118116e-01, -7.321056681877724515e-01, 
    #                        3.828147183491327119e-05, 1.508161050588582202e-03, 
    #                        9.120081467574957523e-01], dtype=np.float32)
    
    # 640 * 480
    mtx = np.array([[532.76634274 ,  0.,         306.22017641],
                    [  0.         , 532.94411762, 225.40097394],
                    [  0.         ,  0.         , 1.        ]], dtype=np.float32)
    
    distCoeffs = np.array([ 0.17156132, -0.66222681, -0.00294354,  0.00106322,  0.73823942], dtype=np.float32)

def doPrepareReq(request):
    print("Received an empty service request")
    # 这里执行你需要的操作
    return std_srvs.EmptyResponse()  # 返回空响应

def talker():
    # 初始化节点，命名为'camera'
    rospy.init_node('camera')
    
    init_camera()
    init_const()
    init_camera_calibration()
    init_empirical_value()

    recognition_node = Recognition_node()
    
    prepare_server = rospy.Service(utilis.Topic_name.camera_prepare_service,std_srvs.Empty,doPrepareReq)
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        rospy.loginfo("camera")
        # 按照设定的频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
