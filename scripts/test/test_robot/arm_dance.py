#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import copy

rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis

from std_msgs.msg import String

import arm
import task
import robot

left_arm_idle  = True
right_arm_idle = True

# 机械臂跳舞

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('arm_dance')
    task_list = []
    
    left_arm_idle                = constant_config_to_arm_anchor_pose("LeftArmIdle",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_container_rec       = constant_config_to_arm_anchor_pose("LeftArmContainerRec",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_snack_rec           = constant_config_to_arm_anchor_pose("LeftArmSnackRec",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_snack_placement     = constant_config_to_arm_anchor_pose("LeftArmSnackPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    left_arm_container_delivery  = constant_config_to_arm_anchor_pose("LeftArmContainerDelivery",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    left_arm_container_placement = constant_config_to_arm_anchor_pose("LeftArmContainerPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    
    right_arm_idle                = constant_config_to_arm_anchor_pose("RightArmIdle",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_container_rec       = constant_config_to_arm_anchor_pose("RightArmContainerRec",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_snack_rec           = constant_config_to_arm_anchor_pose("RightArmSnackRec",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_snack_placement     = constant_config_to_arm_anchor_pose("RightArmSnackPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_container_delivery  = constant_config_to_arm_anchor_pose("RightArmContainerDelivery",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_container_placement = constant_config_to_arm_anchor_pose("RightArmContainerPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    
    right_arm_cup_grab            = constant_config_to_arm_anchor_pose("CupGrab",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_rec             = constant_config_to_arm_anchor_pose("CupRec",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_cup_rec_before      = constant_config_to_arm_anchor_pose("CupRecBefore",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_cup_water           = constant_config_to_arm_anchor_pose("CupWater",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_delivery        = constant_config_to_arm_anchor_pose("CupDelivery",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_placement       = constant_config_to_arm_anchor_pose("CupPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)

    left_arm_coffee_machine_rec           = constant_config_to_arm_anchor_pose("CoffeeMachineRec",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_coffee_machine_open_prepare  = constant_config_to_arm_anchor_pose("CoffeeMachineOpenPrepare",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    left_arm_coffee_machine_close_prepare = constant_config_to_arm_anchor_pose("CoffeeMachineClosePrepare",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    
    # 识别容器
    task_arms_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_container_rec,right_arm_container_rec])
    task_list.append(task_arms_to_rec_contianer)
    
    # 识别零食
    task_arms_to_rec_snack     = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_snack_rec,right_arm_snack_rec])
    task_list.append(task_arms_to_rec_snack)
    
    # 抓取容器
    # task_arms_to_
    
    # 收紧抓爪
    
    # 转移容器
    task_arms_container_delivery = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_container_delivery ,right_arm_container_delivery])
    task_list.append(task_arms_container_delivery)
    
    # 放置容器
    task_arms_container_placement = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_container_placement ,right_arm_container_placement])
    task_list.append(task_arms_container_placement)
    
    # 左,右臂识别杯子前摇
    task_arms_to_rec_cup_machine_before = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_idle, right_arm_cup_rec_before],[robot.manipulation_status.clamp.status.CLOSE,robot.manipulation_status.clamp.status.CLOSE]) 
    task_list.append(task_arms_to_rec_cup_machine_before)
    
    # 识别杯子_machine
    task_arms_to_rec_cup_machine    = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_coffee_machine_rec ,right_arm_cup_rec])
    task_list.append(task_arms_to_rec_cup_machine)
        
    # 夹取杯子 
    task_right_arm_cup_grab         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_grab],robot.manipulation_status.clamp.status.OPEN)
    task_right_arm_cup_grab.set_clamp_first
    task_list.append(task_right_arm_cup_grab)
    
    # 夹取杯子关闭夹子
    task_right_arm_cup_grab_close    = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_grab],robot.manipulation_status.clamp.status.CLOSE)
    task_right_arm_cup_grab_close.set_clamp_first
    task_list.append(task_right_arm_cup_grab_close)
    
    # 开启机器
    task_left_arm_coffee_machine_open_prepare = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [left_arm_coffee_machine_open_prepare])
    task_list.append(task_left_arm_coffee_machine_open_prepare)
    
    
    left_arm_coffee_machine_open = copy.deepcopy(left_arm_coffee_machine_open_prepare)
    left_arm_coffee_machine_open.arm_pose[2] = left_arm_coffee_machine_open.arm_pose[2] + 20
    task_left_arm_coffee_machine_open         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [left_arm_coffee_machine_open])
    task_list.append(task_left_arm_coffee_machine_open)
    
    # 杯子打水
    task_right_arm_cup_water         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_water])
    task_list.append(task_right_arm_cup_water )
    
    # 关闭机器
    task_left_arm_coffee_machine_close_prepare = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [left_arm_coffee_machine_close_prepare])
    task_list.append(task_left_arm_coffee_machine_close_prepare)
    
    
    left_arm_coffee_machine_close = copy.deepcopy(left_arm_coffee_machine_close_prepare)
    left_arm_coffee_machine_close.arm_pose[2] = left_arm_coffee_machine_close.arm_pose[2] - 20
    task_left_arm_coffee_machine_close         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [task_left_arm_coffee_machine_close])
    task_list.append(task_left_arm_coffee_machine_close)
    
    # 移动杯子
    task_right_arm_cup_delivery = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_delivery])
    task_list.append(task_right_arm_cup_delivery)
    
    # 放置杯子
    task_right_arm_cup_placement = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_placement], robot.manipulation_status.clamp.status.OPEN)
    task_list.append(task_right_arm_cup_placement)
    
    # 回归空闲
    task_arms_to_idle       = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_idle ,right_arm_idle],[robot.manipulation_status.clamp.status.CLOSE,robot.manipulation_status.clamp.status.CLOSE])
    task_list.append(task_arms_to_idle)
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("arm")
        # 按照设定的频率延时
        rate.sleep()
        


def constant_config_to_arm_anchor_pose(anchor_point_name,type:arm.PoseType,arm_id:utilis.Device_id):
    arm_pose = arm.Arm_pose()
    if type == arm.PoseType.ANGLE:
        arm_pose.arm_pose = rospy.get_param(f'~{anchor_point_name}/angles')
    elif type == arm.PoseType.BASE_COORDS:
        arm_pose.arm_pose = rospy.get_param(f'~{anchor_point_name}/coords')

    arm_pose.type_id  = type
    arm_pose.arm_id   = arm_id
    rospy.loginfo(f"{anchor_point_name} : type {type}, pose {arm_pose.arm_pose}")
    return arm_pose        


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
