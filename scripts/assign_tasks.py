#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# 官方包
from __future__ import annotations
import rospy
import os
import sys
import rospkg
from geometry_msgs.msg  import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
import copy
from enum import Enum,auto
import time
import std_srvs.srv as std_srvs
import std_msgs.msg  as std_msgs

# 自定义包
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import run_task.msg as msg
import task
import utilis
import robot
import order
import log
import arm
import control_cmd

DEBUG_NAVIGATION = False     # 导航调试中, 则运动到桌子的任务均为非并行任务, 并且在完成之后需要输入任意字符才能下一步

# WAIT_FOR_ACTION_SERVER = False       # 是否等待服务器
WAIT_FOR_ACTION_SERVER = True       # 是否等待服务器

# 初始化
class System():
    instance = None          # 句柄
    # 初始化
    def __init__(self):
        System.instance = self
        self.anchor_point = self.Anchor_point()  # 固定参数读取
        
        rospy.loginfo(f"node: {rospy.get_name()}, system init")
        # 执行器初始化
        rospy.loginfo(f"node: {rospy.get_name()}, navigation_actuator")
        self.navigation_actuator  =  Navigation_actuator()  # 导航执行器
        rospy.loginfo(f"node: {rospy.get_name()}, image_rec_actuator")
        self.image_rec_actuator   =  Image_rec_actuator()   # 图像识别执行器
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator_actuator")
        self.manipulator_actuator =  Manipulator_actuator() # 机械臂执行器
        rospy.loginfo(f"node: {rospy.get_name()}, voice_recognition_actuator")
        self.asr_actuator         =  ASR_actuator()  # 语音识别执行器
        
        # 机器人状态
        self.robot                =  robot.Robot()           # 机器人
        
        # 任务管理器
        self.task_manager = Task_manager(self.robot)
        
        # 订单驱动任务增加器
        self.order_driven_task_schedul     = Order_driven_task_schedul(self.task_manager)
        
        # 设置初始位姿
        # TODO:调试需要,暂时注释
        # self.set_initial_pose()
        
        
    
    # 设置初始位姿
    def set_initial_pose(self):
        self.set_amcl_pose(self.anchor_point.map_initial_pose)
    
    # 初始化AMCL位姿
    def set_amcl_pose(self,pose:utilis.Pose3D):
        rospy.loginfo(f"node: {rospy.get_name()} : set initial pose {pose}")
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        # 等待发布者成为有效的
        rospy.sleep(1)
        # 创建PoseWithCovarianceStamped消息
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        initial_pose.pose.pose.position.x = pose.x
        initial_pose.pose.pose.position.y = pose.y
        initial_pose.pose.pose.position.z = 0
        yaw = pose.theta
        q = quaternion_from_euler(0, 0, yaw)
        initial_pose.pose.pose.orientation.x = q[0]
        initial_pose.pose.pose.orientation.y = q[1]
        initial_pose.pose.pose.orientation.z = q[2]
        initial_pose.pose.pose.orientation.w = q[3]

        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[6 * 1 + 1] = 0.25
        initial_pose.pose.covariance[6 * 5 + 5] = 0.06853891945200942
    
        rospy.loginfo(f"node: {rospy.get_name()}, set_amcl_pose position {initial_pose.pose.pose.position} orientation: {q}")
        # 发布初始位置
        pub.publish(initial_pose)
        
    # 常量定义
    class Anchor_point():
        def __init__(self):
            self._initialize_robot_anchor_point()
            self._initialize_arm_anchor_point()
            self._initialize_other_config()
            self._initialize_use_time()
        
        # 特殊的使用时间
        def _initialize_use_time(self):
            self.time_wait_for_turn_on_machine  = rospy.get_param(f'~time_wait_for_turn_on_machine')
            self.time_wait_for_turn_off_machine = rospy.get_param(f'~time_wait_for_turn_off_machine')
            self.time_before_get_image          = rospy.get_param(f'~time_before_get_image')
            self.time_interval_for_task         = rospy.get_param(f'~time_interval_for_task')
            
        # 后退距离
        def _initialize_other_config(self):
            self.snack_deck_move_forward_pose =  self._get_control_cmd_x_yaw_time("SnackDeckMoveforward")
            self.drink_deck_move_forward_pose =  self._get_control_cmd_x_yaw_time("DrinkDeckMoveforward")
            self.left_deck_move_forward_pose  =  self._get_control_cmd_x_yaw_time("LeftDeckMoveforward")
            self.right_deck_move_forward_pose =  self._get_control_cmd_x_yaw_time("RightDeckMoveforward")
            
            self.snack_deck_move_back_pose =  self._get_control_cmd_x_yaw_time("SnackDeckMoveBack")
            self.drink_deck_move_back_pose =  self._get_control_cmd_x_yaw_time("DrinkDeckMoveBack")
            self.left_deck_move_back_pose  =  self._get_control_cmd_x_yaw_time("LeftDeckMoveBack")
            self.right_deck_move_back_pose =  self._get_control_cmd_x_yaw_time("RightDeckMoveBack")
            
        def _get_control_cmd_x_yaw_time(self,name):
            target_pose     = utilis.Pose3D()
            target_pose.x   = rospy.get_param(f'~{name}/x')
            target_pose.yaw = rospy.get_param(f'~{name}/yaw')
            target_pose.run_time = rospy.get_param(f'~{name}/time')
            return target_pose
        
        # 初始化机器人位点常量配置
        def _initialize_robot_anchor_point(self):
            # 初始点
            self.map_initial_pose =  self._get_robot_anchor_pose("InitialPose")
            # 零食桌
            self.map_snack_desk   =  self._get_robot_anchor_pose("SnackDesk")
            # 饮料桌
            self.map_drink_desk   =  self._get_robot_anchor_pose("DrinkDesk")
            # 右服务台位姿
            self.map_right_service_desk =  self._get_robot_anchor_pose("RightServiceDesk")
            # 左服务台位姿
            self.map_left_service_desk  =  self._get_robot_anchor_pose("LeftServiceDesk")
            
        # 通过名字获取机器人定位点
        def _get_robot_anchor_pose(self,anchor_point_name):
            x = rospy.get_param(f'~{anchor_point_name}/position_x')
            y = rospy.get_param(f'~{anchor_point_name}/position_y')
            z = rospy.get_param(f'~{anchor_point_name}/position_z')
            o_x = rospy.get_param(f'~{anchor_point_name}/orientation_x')
            o_y = rospy.get_param(f'~{anchor_point_name}/orientation_y')
            o_z = rospy.get_param(f'~{anchor_point_name}/orientation_z')
            o_w = rospy.get_param(f'~{anchor_point_name}/orientation_w')
            pose = utilis.Pose3D.instantiate_by_xyz_orientation(x,y,z,o_x,o_y,o_z,o_w)
            return pose
        
        # 初始化机械臂位点常量配置
        def _initialize_arm_anchor_point(self):
            self.left_arm_idle                 = self._get_arm_anchor_angle("LeftArmIdle")                # 左臂闲置
            self.left_arm_container_rec        = self._get_arm_anchor_angle("LeftArmContainerRec")        # 左臂识别容器
            self.left_arm_snack_rec            = self._get_arm_anchor_angle("LeftArmSnackRec")            # 左臂零食识别
            self.left_arm_snack_grip_pre       = self._get_arm_anchor_coord("LeftArmGripSnackPre")        # 左臂零食抓取准备点
            self.left_arm_snack_grip           = self._get_arm_anchor_coord("LeftArmGripSnack")           # 左臂零食抓取
            self.left_arm_snack_placement_pre  = self._get_arm_anchor_coord("LeftArmSnackPlacementPre")   # 左臂零食放置准备点
            self.left_arm_snack_placement      = self._get_arm_anchor_coord("LeftArmSnackPlacement")      # 左臂零食放置
            self.left_arm_container_grip_dodge = self._get_arm_anchor_coord("LeftArmGripContainerDodge")  # 左臂抓取容器前, 躲避容器的位姿
            self.left_arm_container_grip_pre   = self._get_arm_anchor_coord("LeftArmGripContainerPre")    # 左臂抓取容器准备状态
            self.left_arm_container_grip       = self._get_arm_anchor_coord("LeftArmGripContainer")       # 左臂抓取容器
            self.left_arm_container_delivery   = self._get_arm_anchor_coord("LeftArmContainerDelivery")   # 左臂容器运送时的姿态
            self.left_arm_container_placement  = self._get_arm_anchor_coord("LeftArmContainerPlacement")  # 左臂容器放置
            self.left_arm_machine_turn_on_rec  = self._get_arm_anchor_angle("LeftArmMachineTurnOnRec")    # 左臂 开 咖啡机识别
            self.left_arm_machine_turn_off_rec = self._get_arm_anchor_angle("LeftArmMachineTurnOFFRec")   # 左臂 关 咖啡机识别
            self.left_arm_machine_turn_on_pre  = self._get_arm_anchor_coord("LeftArmMachineTurnOnPre")    # 左臂 开 咖啡机预备动作
            self.left_arm_machine_turn_on_click= self._get_arm_anchor_coord("LeftArmMachineTurnOnClick")  # 左臂 开 咖啡机 向上拨一拨
            self.left_arm_machine_transfrom    = self._get_arm_anchor_coord("LeftArmMachineTransfrom")    # 左臂 咖啡机位置转换
            self.left_arm_machine_turn_off_pre = self._get_arm_anchor_coord("LeftArmMachineTurnOffPre")   # 左臂 关 咖啡机预备动作
            self.left_arm_machine_turn_off_click= self._get_arm_anchor_coord("LeftArmMachineTurnOffClick")# 左臂 关 咖啡机 向下拨一拨
            
            
            self.right_arm_idle                = self._get_arm_anchor_angle("RightArmIdle")               # 右臂空闲
            self.right_arm_container_rec       = self._get_arm_anchor_angle("RightArmContainerRec")       # 右臂识别容器
            self.right_arm_snack_rec           = self._get_arm_anchor_angle("RightArmSnackRec")           # 右臂识别零食
            self.right_arm_snack_grip_pre      = self._get_arm_anchor_angle("RightArmGripSnackPre")       # 右臂零食抓取准备点
            self.right_arm_snack_grip          = self._get_arm_anchor_coord("RightArmGripSnack")          # 右臂零食抓取
            self.right_arm_snack_placement_pre = self._get_arm_anchor_coord("RightArmSnackPlacementPre")  # 右臂零食放置准备点
            self.right_arm_snack_placement     = self._get_arm_anchor_coord("RightArmSnackPlacement")     # 右臂零食放置
            self.right_arm_container_grip_dodge= self._get_arm_anchor_coord("RightArmGripContainerDodge") # 右臂抓取容器前, 躲避容器的位姿
            self.right_arm_container_grip_pre  = self._get_arm_anchor_coord("RightArmGripContainerPre")   # 右臂抓取容器准备状态
            self.right_arm_container_grip      = self._get_arm_anchor_coord("RightArmGripContainer")      # 右臂抓取容器
            self.right_arm_container_delivery  = self._get_arm_anchor_coord("RightArmContainerDelivery")  # 右臂容器运送时的姿态
            self.right_arm_container_placement = self._get_arm_anchor_coord("RightArmContainerPlacement") # 右臂容器放置
            self.right_arm_cup_rec_pre         = self._get_arm_anchor_angle("RightArmCupRecPre")          # 右臂杯子识别预备
            self.right_arm_cup_rec             = self._get_arm_anchor_angle("RightArmCupRec")             # 右臂杯子识别
            self.right_arm_cup_grab_pre        = self._get_arm_anchor_coord("RightArmCupGrabPre")         # 右臂杯子夹取, 准备点
            self.right_arm_cup_grab            = self._get_arm_anchor_coord("RightArmCupGrab")            # 右臂杯子夹取
            self.right_arm_cup_water           = self._get_arm_anchor_coord("RightArmCupWater")           # 右臂杯子接水
            self.right_arm_cup_delivery        = self._get_arm_anchor_coord("RightArmCupDelivery")        # 右臂杯子运送
            self.right_arm_cup_placement       = self._get_arm_anchor_coord("RightArmCupPlacement")       # 右臂杯子放置

        # 通过名字获取机械臂定位点
        def _get_arm_anchor_coord(self,anchor_point_name):
            if "left" in  anchor_point_name.lower() :
                arm_id = utilis.Device_id.LEFT
            elif "right" in  anchor_point_name.lower() :
                arm_id = utilis.Device_id.RIGHT
            else:
                arm_id = utilis.Device_id.TBD
            pose = arm.Arm_pose(rospy.get_param(f'~{anchor_point_name}/coords'),arm.PoseType.BASE_COORDS,arm_id)
            return pose
        
        # 通过名字获取机械臂定位点(角度)
        def _get_arm_anchor_angle(self,anchor_point_name):
            if "left" in  anchor_point_name.lower() :
                arm_id = utilis.Device_id.LEFT
            elif "right" in  anchor_point_name.lower() :
                arm_id = utilis.Device_id.RIGHT
            else:
                arm_id = utilis.Device_id.TBD
            pose = arm.Arm_pose(rospy.get_param(f'~{anchor_point_name}/angles'),arm.PoseType.ANGLE,arm_id)
            return pose
        

# 导航任务执行器
class Navigation_actuator():
    # 初始化
    def __init__(self):
        # 订阅导航Action   
        self.move_base_ac   = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.control_cmd_ac = actionlib.SimpleActionClient(utilis.Topic_name.control_cmd_action, msg.ControlCmdAction)
        if WAIT_FOR_ACTION_SERVER:
            rospy.loginfo("waiting for move_base")
            # TODO:调试需要,暂时注释
            self.move_base_ac.wait_for_server()
            rospy.loginfo("waiting for control cmd server")
            self.control_cmd_ac.wait_for_server()
        
        self.running_tasks_manager = task.Task_manager_in_running() # 正在执行的任务管理器
        
    # 运行
    def run(self, navigation_task:task.Task_navigation):
        task_index = self.running_tasks_manager.add_task(navigation_task)
        navigation_task.update_start_status() # 刷新开始时间
        system.robot.robot_status = robot.Robot.Robot_status.MOVING  # 机器人状态更新

        # 任务开始暂停时间
        if navigation_task.sleep_time_before_task != 0:
            time.sleep(navigation_task.sleep_time_before_task)
            rospy.loginfo(f"task {task_index} sleep for {navigation_task.sleep_time_before_task} second before task")
        
        # 后退任务
        if navigation_task.task_type == task.Task_type.Task_navigate.Move_backward or \
            navigation_task.task_type == task.Task_type.Task_navigate.Move_forward:
            goal = msg.ControlCmdGoal()
            goal.task_index = task_index
            goal.operation  = control_cmd.Control_cmd.MOVEBACK.value
            goal.second     = navigation_task.move_back_second         # 后退秒数
            goal.x          = navigation_task.target_3D_pose.x         # 后退距离x
            goal.yaw        = navigation_task.target_3D_pose.yaw       # 后退距离yaw
            self.control_cmd_ac.send_goal(goal,self.control_cmd_task_done_callback,self.control_cmd_active_callback,self.control_cmd_feedback_callback)
        # 导航任务
        else:
            self.move_base_task_index = task_index
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp    = rospy.Time.now()
            goal.target_pose.pose.position.x = navigation_task.target_3D_pose.x
            goal.target_pose.pose.position.y = navigation_task.target_3D_pose.y
            goal.target_pose.pose.position.z = 0
            orientation = quaternion_from_euler(0, 0, navigation_task.target_3D_pose.yaw)
            goal.target_pose.pose.orientation.x = orientation[0]
            goal.target_pose.pose.orientation.y = orientation[1]
            goal.target_pose.pose.orientation.z = orientation[2]
            goal.target_pose.pose.orientation.w = orientation[3]
            self.move_base_ac.send_goal(goal,self.navigation_task_done_callback,self.navigation_task_active_callback,self.navigation_task_feedback_callback)

    # 完成回调
    @staticmethod
    def navigation_task_done_callback(status, result):
        rospy.loginfo(f"node: {rospy.get_name()}, navigation done. status:{status} result:{result}")
        current_task = system.navigation_actuator.running_tasks_manager.get_task(system.navigation_actuator.move_base_task_index)
        
        # 任务完成暂停时间
        if current_task.sleep_time_after_task != 0:
            time.sleep(current_task.sleep_time_after_task)
            rospy.loginfo(f"task {current_task.task_index} sleep for {current_task.sleep_time_after_task} second after task")
        else:
            rospy.loginfo(f"task {current_task.task_index} not sleep after task ")
        

        rospy.loginfo(f"node: {rospy.get_name()}, navigation finish. status : {status}")
        current_task.update_end_status(task.Task.Task_result.SUCCEED)
        
        # 任务自带的回调
        if current_task.finish_cb is not None:
            current_task.finish_cb(status, result)
        
        if DEBUG_NAVIGATION:
            input("The navigation point has been reached, whether to continue running ")
            
        # 给任务管理器的回调
        system.task_manager.tm_task_finish_callback(current_task, status, result)
    
    # 激活回调
    @staticmethod
    def navigation_task_active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, navigation active")

    # 反馈回调
    @staticmethod
    def navigation_task_feedback_callback(feedback:MoveBaseFeedback):
        pose = feedback.base_position.pose
        pose3D = utilis.Pose3D.instantiate_by_geometry_msg(pose)
        # rospy.loginfo(f"node: {rospy.get_name()}, navigation feedback. pose:x = {pose3D.x} y = {pose3D.y} yaw = {pose3D.yaw}")

    # 直接控制完成回调
    @staticmethod
    def control_cmd_task_done_callback(status, result:msg.ControlCmdResult):
        rospy.loginfo(f"node: {rospy.get_name()}, control cmd task done. status:{status} result:{result}")
        current_task = system.navigation_actuator.running_tasks_manager.get_task(result.task_index)
        
        # 任务自带的回调
        if current_task.finish_cb is not None:
            current_task.finish_cb(status, result)
            
        # 任务完成暂停时间
        if current_task.sleep_time_after_task != 0:
            time.sleep(current_task.sleep_time_after_task)
            rospy.loginfo(f"task index {current_task.task_index} sleep for {current_task.sleep_time_after_task} second after task")
        else:
            rospy.loginfo(f"task index {current_task.task_index} not sleep after task")
        
        rospy.loginfo(f"node: {rospy.get_name()}, control cmd finish. status : {status}")
        current_task.update_end_status(task.Task.Task_result.SUCCEED)

            
        # 给任务管理器的回调
        system.task_manager.tm_task_finish_callback(current_task, status, result)
    
    # 激活回调
    @staticmethod
    def control_cmd_active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, control cmd active")

    # 反馈回调
    @staticmethod
    def control_cmd_feedback_callback(feedback:MoveBaseFeedback):
        pass 
    
# 机械臂执行器 
class Manipulator_actuator():
    def __init__(self):
        global left_arm_pub,right_arm_pub
        # 手臂任务发布 
        left_arm_pub  = rospy.Publisher(utilis.Topic_name.left_arm_topic, msg.ArmMoveRequest, queue_size=10)
        right_arm_pub = rospy.Publisher(utilis.Topic_name.right_arm_topic,msg.ArmMoveRequest, queue_size=10)
        
        # 手臂结果接受
        left_arm_sub  = rospy.Subscriber(utilis.Topic_name.left_arm_result,msg.ArmMoveResult ,do_left_arm_move_result, queue_size=10)
        right_arm_sub = rospy.Subscriber(utilis.Topic_name.right_arm_result,msg.ArmMoveResult,do_right_arm_move_result,queue_size=10)
        
        self.running_tasks_manager = task.Task_manager_in_running() # 正在执行的任务管理器
    
    # 运行
    def run(self, manipulation_task:task.Task_manipulation):
        # 加在运行序列中
        task_index = self.running_tasks_manager.add_task(manipulation_task)
        # 更新手臂状态
        system.robot.update_arm_status(manipulation_task.arm_id,robot.manipulation_status.arm.status.BUSY)
        # !如果是放零食任务, 则需要更新放零食位置的状态
        if manipulation_task.task_type == task.Task_type.Task_manipulation.Lossen_snack:
            system.robot.lossen_snack_point_status = robot.Robot.Common_status.BUSY

        rospy.loginfo(f"manipulation task index {manipulation_task.task_index} is running ")
        
        # 任务开始
        manipulation_task.update_start_status()
        
        # 任务前休眠时间
        if manipulation_task.sleep_time_before_task != 0:
            time.sleep(manipulation_task.sleep_time_before_task)
            rospy.loginfo(f"task index {task_index} sleep for {manipulation_task.sleep_time_before_task} second before task")

        # 左臂
        if manipulation_task.arm_id == utilis.Device_id.LEFT:
            # 设置手臂 目标
            left_goal                      = msg.ArmMoveRequest()
            left_goal.task_index           = task_index
            left_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0].arm_pose
            left_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            left_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            left_goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            left_goal.grasp_speed          = manipulation_task.clamp_speed
            left_goal.arm_move_method      = manipulation_task.arm_move_method.value
            left_goal.arm_id               = manipulation_task.target_arms_pose[0].arm_id.value
            
            rospy.loginfo(f"left_arm_pub type {type(left_arm_pub)}")
            left_arm_pub.publish(left_goal)
            rospy.loginfo(f"left_arm_pub send goal")
        # 右臂
        elif manipulation_task.arm_id == utilis.Device_id.RIGHT:
            # 设置action 目标
            right_goal                      = msg.ArmMoveRequest()
            right_goal.task_index           = task_index
            right_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0].arm_pose
            right_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            right_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            right_goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            right_goal.grasp_speed          = manipulation_task.clamp_speed
            right_goal.arm_move_method      = manipulation_task.arm_move_method.value
            right_goal.arm_id               = manipulation_task.target_arms_pose[0].arm_id.value
            right_arm_pub.publish(right_goal)
            rospy.loginfo(f"right_arm_pub send goal")
            
        elif manipulation_task.arm_id == utilis.Device_id.LEFT_RIGHT:
            left_goal              = msg.ArmMoveRequest()
            right_goal             = msg.ArmMoveRequest()
            for i in range(2):
                if manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.LEFT:
                    left_goal.task_index           = task_index
                    left_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i].arm_pose
                    left_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    left_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    left_goal.grasp_flag           = manipulation_task.target_clamps_status[i].value
                    left_goal.grasp_speed          = manipulation_task.clamp_speed
                    left_goal.arm_move_method      = manipulation_task.arm_move_method.value
                    left_goal.arm_id               = manipulation_task.target_arms_pose[i].arm_id.value
                elif manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.RIGHT:
                    right_goal.task_index           = task_index
                    right_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i].arm_pose
                    right_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    right_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    right_goal.grasp_flag           = manipulation_task.target_clamps_status[i].value
                    right_goal.grasp_speed          = manipulation_task.clamp_speed
                    right_goal.arm_move_method      = manipulation_task.arm_move_method.value
                    right_goal.arm_id               = manipulation_task.target_arms_pose[i].arm_id.value

            left_arm_pub.publish(left_goal)
            right_arm_pub.publish(right_goal)
            rospy.loginfo(f"right_arm_pub send goal")
            rospy.loginfo(f"left_arm_pub send goal")


# 左臂接受结果信息
def do_left_arm_move_result(result:msg.ArmMoveResult):
    rospy.loginfo(f"left arm move get result : task index {result.task_index}")

    current_task =  system.manipulator_actuator.running_tasks_manager.get_task(result.task_index)
    # !如果是夹容器的躲避动作, 则需要更新放零食位置的状态 
    
    if current_task.task_type == task.Task_type.Task_manipulation.Grasp_container_dodge:
        system.robot.lossen_snack_point_status = robot.Robot.Common_status.IDLE
    
    # 任务完成暂停时间
    if current_task.sleep_time_after_task != 0:
        time.sleep(current_task.sleep_time_after_task)
        rospy.loginfo(f"task index {current_task.task_index} sleep for {current_task.sleep_time_after_task} second after task")
    else:
        rospy.loginfo(f"task index {current_task.task_index} not sleep")
        
    try:
        current_task.update_end_status(task.Task.Task_result.SUCCEED)
    except Exception as e:
        rospy.logerr(f"Exception in done_cb: {e}")
        current_task.update_end_status(task.Task.Task_result.SUCCEED)
        
    # 任务自带的回调
    if current_task.finish_cb != None:
        current_task.finish_cb(None, result)
    
    # 删除任务
    # 有可能同一个任务被删除两次
    try:
        if current_task.if_finished():
            rospy.loginfo(f"del task index {result.task_index} in running_tasks_manager OK")
            system.manipulator_actuator.running_tasks_manager.del_task(result.task_index)
        else :
            rospy.loginfo(f"task index{result.task_index} is not finished, keep in running list")
    except:
        rospy.loginfo(f"!!!!!!!!!!!!!!!!!!!!del task index {result.task_index} in running_tasks_manager error")
    
    # 给任务管理器的回调
    system.task_manager.tm_task_finish_callback(current_task, None, result)


# 右臂接受结果信息
def do_right_arm_move_result(result:msg.ArmMoveResult):
    rospy.loginfo(f"right arm move get result : task index {result.task_index}")
    current_task =  system.manipulator_actuator.running_tasks_manager.get_task(result.task_index)
    
    # !如果是夹容器的准备动作, 则需要更新放零食位置的状态 
    if current_task.task_type == task.Task_type.Task_manipulation.Grasp_container_dodge:
        system.robot.lossen_snack_point_status = robot.Robot.Common_status.IDLE
        
    # 任务完成暂停时间
    if current_task.sleep_time_after_task != 0:
        time.sleep(current_task.sleep_time_after_task)
        rospy.loginfo(f"task index {current_task.task_index} sleep for {current_task.sleep_time_after_task} second after task")
    else:
        rospy.loginfo(f"task index {current_task.task_index} not sleep")
        
    try:
        current_task.update_end_status(task.Task.Task_result.SUCCEED)
    except Exception as e:
        rospy.logerr(f"Exception in done_cb: {e}")
        current_task.update_end_status(task.Task.Task_result.SUCCEED)
    # 任务自带的回调
    if current_task.finish_cb != None:
        current_task.finish_cb(None, result)

        
    # 删除任务
    try:
        if current_task.if_finished():
            rospy.loginfo(f"del task index {result.task_index} in running_tasks_manager OK")
            system.manipulator_actuator.running_tasks_manager.del_task(result.task_index)
        else :
            rospy.loginfo(f"task index {result.task_index} is not finished, keep in running list")
    except:
        rospy.loginfo(f"!!!!!!!!!!!!!!!!!!!!del task index {result.task_index} in running_tasks_manager error")
    
    # 给任务管理器的回调
    system.task_manager.tm_task_finish_callback(current_task, None, result)
    

# 图像识别任务执行器
class Image_rec_actuator():
    def __init__(self):
        self.running_tasks_manager  = task.Task_manager_in_running()  # 正在执行的任务管理器       
        self.pub = rospy.Publisher (utilis.Topic_name.image_recognition_request ,msg.ImageRecRequest,queue_size=10) # 发布识别任务
        self.sub = rospy.Subscriber(utilis.Topic_name.image_recognition_result  ,msg.ImageRecResult ,Image_rec_actuator.do_image_rec_result_callback,queue_size=10)      # 订阅识别结果

    # 运行
    def run(self, task_image_rec_task:task.Task_image_rec):
        task_index = self.running_tasks_manager.add_task(task_image_rec_task)
        # 任务开始
        task_image_rec_task.update_start_status()
        # 更新机械臂状态
        system.robot.update_arm_status(task_image_rec_task.camera_id,robot.manipulation_status.arm.status.BUSY)
        
        # 任务前休眠时间
        if task_image_rec_task.sleep_time_before_task != 0:
            time.sleep(task_image_rec_task.sleep_time_before_task)
            rospy.loginfo(f"task index {task_index} sleep for {task_image_rec_task.sleep_time_before_task} second before task")
            
        # 发布任务
        task_info = msg.ImageRecRequest()
        task_info.task_index = task_index                                    # 任务索引
        task_info.task_type  = task_image_rec_task.task_type.task_type.value # 任务类型
        task_info.camera_id  = task_image_rec_task.camera_id.value           # 相机ID
        # 如果是识别零食, 则需要给出零食列表
        if task_info.task_type == task.Task_type.Task_image_rec.SNACK:
            task_info.snacks = task_image_rec_task.snack_list.to_list()      # 零食列表
        # 发布消息
        self.pub.publish(task_info)

    # 识别结果话题回调
    @staticmethod
    def do_image_rec_result_callback(result:msg.ImageRecResult):
        rospy.loginfo(f"do_image_rec_result {result}")
        # 获取对应的服务对象
        current_task:task.Task_image_rec = system.image_rec_actuator.running_tasks_manager.get_task(result.task_index)
        # 根据不同任务作出不同处理
        # 识别零食
        if current_task.task_type.task_type == task.Task_type.Task_image_rec.SNACK:
            rospy.loginfo("==!!!!!!!")
            rec_snack_count = len(result.obj_positions)
            try:
                rospy.loginfo(f"rec_snack_count {rec_snack_count}")
                for i in range(rec_snack_count):
                    rospy.loginfo(f"obj_positions[{i}]:{ result.obj_positions[i]}")
                    snack_xyz                = result.obj_positions[i].position
                    arm_id                   = result.obj_positions[i].arm_id
                    task_grasp_snack_pre:task.Task_manipulation     = current_task.need_modify_tasks.task_list[i*6]   # 抓零食准备任务
                    task_grasp_snack_pre.select_arm(arm_id)
                    task_grasp_snack:task.Task_manipulation         = current_task.need_modify_tasks.task_list[i*6+1] # 抓零食任务
                    task_grasp_snack.modify_xyz_select_arm(snack_xyz,arm_id)
                    task_lossen_snack_pre:task.Task_manipulation    = current_task.need_modify_tasks.task_list[i*6+2] # 松零食准备任务

                    if arm_id == utilis.Device_id.LEFT:
                        # 左臂松零食准备动作使用 Z_X_Y
                        # 左臂夹下层零食用 XY_Z
                        if snack_xyz[2] < 500:
                            task_lossen_snack_pre.select_arm(arm_id,arm.ArmMoveMethod.Z_X_Y)
                            task_grasp_snack.arm_move_method = arm.ArmMoveMethod.XY_Z
                        else:
                            task_lossen_snack_pre.select_arm(arm_id)
                    else:
                        task_lossen_snack_pre.select_arm(arm_id)
                    task_lossen_snack:task.Task_manipulation        = current_task.need_modify_tasks.task_list[i*6+3]  # 松零食任务
                    task_lossen_snack.select_arm(arm_id)
                    
                    task_left_arm_grap_container_pre :task.Task_manipulation  = current_task.need_modify_tasks.task_list[i*6+4] # 左臂夹取零食框的准备动作
                    task_right_arm_grap_container_pre:task.Task_manipulation  = current_task.need_modify_tasks.task_list[i*6+5] # 右臂夹取零食框的准备动作
                    
                    # 左臂则, 左臂夹取零食框的准备动作需要等放零食结束. 而右臂不需要
                    if arm_id == utilis.Device_id.LEFT:
                        task_right_arm_grap_container_pre.del_prodecessor_task(task_lossen_snack)
                    elif arm_id == utilis.Device_id.RIGHT:
                        task_left_arm_grap_container_pre.del_prodecessor_task(task_lossen_snack)
                    
                    task_grasp_snack_pre.status  = task.Task.Task_status.BEREADY
                    task_grasp_snack.status      = task.Task.Task_status.BEREADY
                    task_lossen_snack_pre.status = task.Task.Task_status.BEREADY
                    task_lossen_snack.status     = task.Task.Task_status.BEREADY
            except:
                rospy.loginfo("!!!!snack count is not equal to task count")
            
            # 没有识别到的零食, 删除需要任务里的前置任务
            req_snack_count = current_task.get_snack_count()
            if rec_snack_count < req_snack_count:
                for i in range(rec_snack_count,req_snack_count):
                    task_left_arm_grap_container_pre :task.Task_manipulation  = current_task.need_modify_tasks.task_list[i*6+4] # 左臂夹取零食框的准备动作
                    task_right_arm_grap_container_pre:task.Task_manipulation  = current_task.need_modify_tasks.task_list[i*6+5] # 右臂夹取零食框的准备动作
                    task_lossen_snack:task.Task_manipulation                  = current_task.need_modify_tasks.task_list[i*6+3]  # 松零食任务
                    task_right_arm_grap_container_pre.del_prodecessor_task(task_lossen_snack)  # 删除前置任务
                    task_left_arm_grap_container_pre.del_prodecessor_task(task_lossen_snack)   # 删除前置任务
                    
        
        # 识别容器
        elif current_task.task_type.task_type == task.Task_type.Task_image_rec.CONTAINER:
            # 获取结果
            for obj_position in result.obj_positions:
                if obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.CONTAINER_LEFT or\
                    obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.CONTAINER_RIGHT :
                    # 抓取容器区域
                    container_xyz = obj_position.position
                elif obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.LOSSEN_SNACK:
                    # 放置零食区域
                    lossen_snack_xyz = obj_position.position
                elif obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.CONTAINER_DODGE:
                    # 躲避容器区域
                    dodge_container_xyz = obj_position.position
            
            # 修改值 
            for need_modify_task in current_task.need_modify_tasks.task_list:
                # 松开零食
                if need_modify_task.task_type == task.Task_type.Task_manipulation.Lossen_snack:
                    # 改变xy()
                    need_modify_task.modify_target_xy(lossen_snack_xyz,result.camera_id)
                    # !!!不改变任务状态
                # 抓容器, 变的是xy坐标
                elif need_modify_task.task_type == task.Task_type.Task_manipulation.Grasp_container_pre:
                    need_modify_task.modify_target_xy(container_xyz,result.camera_id)
                    # 修改任务状态
                    need_modify_task.status = task.Task.Task_status.BEREADY
                # 躲避容器, 变的是xy坐标
                elif need_modify_task.task_type == task.Task_type.Task_manipulation.Grasp_container_dodge:
                    need_modify_task.modify_target_xy(dodge_container_xyz,result.camera_id)

        # 识别咖啡机, 开机 or 关机
        elif current_task.task_type.task_type == task.Task_type.Task_image_rec.COFFEE_MACHINE_SWITCH_ON :
            # 获取结果
            # TODO:待修改
            for obj_position in result.obj_positions:
                # 开机
                if obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_ON:
                    switch_on_xyz = obj_position.position
                # 关机
                elif obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_OFF:
                    switch_off_xyz = obj_position.position
                elif obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH_TRA:
                    switch_tra_xyz = obj_position.position
                # 接水点
                elif obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.WATER_POINT:
                    water_xyz = obj_position.position
                    
            # 修改值 
            for need_modify_task in current_task.need_modify_tasks.task_list:
                # 打开或关闭咖啡机
                if need_modify_task.task_type == task.Task_type.Task_manipulation.Turn_on_coffee_machine:
                    need_modify_task.modify_target_xyz(switch_on_xyz,result.camera_id)
                    # 修改任务状态
                    need_modify_task.status = task.Task.Task_status.BEREADY
                elif need_modify_task.task_type == task.Task_type.Task_manipulation.Turn_off_coffee_machine:
                    need_modify_task.modify_target_xyz(switch_off_xyz,result.camera_id)
                    # 修改任务状态
                    need_modify_task.status = task.Task.Task_status.BEREADY
                elif need_modify_task.task_type == task.Task_type.Task_manipulation.Turn_tra_coffee_machine:
                    need_modify_task.modify_target_xyz(switch_tra_xyz,result.camera_id)
                    # 修改任务状态
                    need_modify_task.status = task.Task.Task_status.BEREADY
                elif need_modify_task.task_type == task.Task_type.Task_manipulation.Water_cup:
                    need_modify_task.modify_target_xyz(water_xyz,result.camera_id)
                    # 修改任务状态
                    need_modify_task.status = task.Task.Task_status.BEREADY

        # 识别杯子
        elif current_task.task_type.task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE:
            # 获取结果
            for obj_position in result.obj_positions:
                if obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.CUP:
                    cup_xyz = obj_position.position
                elif obj_position.obj_id == task.Task_image_rec.Rec_OBJ_type.WATER_POINT:
                    water_xyz = obj_position.position
            # 修改值 
            for need_modify_task in current_task.need_modify_tasks.task_list:
                if need_modify_task.task_type == task.Task_type.Task_manipulation.Grasp_cup:
                    # 抓杯子
                    need_modify_task.modify_target_xyz(cup_xyz,result.camera_id)
                    need_modify_task.status = task.Task.Task_status.BEREADY
                elif need_modify_task.task_type == task.Task_type.Task_manipulation.Grasp_cup_pre:
                    # 抓杯子 准备时, 只改变yz
                    need_modify_task.modify_target_yz(cup_xyz,result.camera_id)
                    need_modify_task.status = task.Task.Task_status.BEREADY
                elif need_modify_task.task_type == task.Task_type.Task_manipulation.Water_cup:
                    # 给杯子浇水
                    need_modify_task.modify_target_xyz(water_xyz,result.camera_id)
                    need_modify_task.status = task.Task.Task_status.BEREADY
                    
        else:
            raise ValueError("Invalid task type")

        # 任务完成暂停时间
        if current_task.sleep_time_after_task != 0:
            time.sleep(current_task.sleep_time_after_task)
            rospy.loginfo(f"task index {result.task_index} sleep for {current_task.sleep_time_after_task} second after task")
        else:
            rospy.loginfo(f"task  index {result.task_index} not sleep after task")
            
        # 更新任务状态
        current_task.update_end_status(task.Task.Task_result.SUCCEED)
        # 任务自带的回调
        if current_task.finish_cb is not None:
            pass

        # 给任务管理器的回调
        system.task_manager.tm_task_finish_callback(current_task)
    

# !语音识别任务执行器
class ASR_actuator():
    def __init__(self):
        # 语音识别请求
        self.asr_request_pub = rospy.Publisher(utilis.Topic_name.asr_request,std_msgs.Empty,queue_size=10)

    def run(self,asr_task:task.Task_speech_recognition):
        current_task = asr_task
        # 任务开始
        asr_task.update_start_status()

        # 任务前休眠时间
        if asr_task.sleep_time_before_task != 0:
            time.sleep(asr_task.sleep_time_before_task)
            rospy.loginfo(f"task index {asr_task.task_index} sleep for {asr_task.sleep_time_before_task} second before task")
        
        # 发送识别请求
        self.asr_request_pub.publish(std_msgs.Empty())
        rospy.loginfo("send asr request")
        
        # 任务完成暂停时间
        if current_task.sleep_time_after_task != 0:
            time.sleep(current_task.sleep_time_after_task)
        else:
            pass
        
        # 更新任务状态
        current_task.update_end_status(task.Task.Task_result.SUCCEED)
        # 任务自带的回调
        if current_task.finish_cb is not None:
            pass
        
        system.task_manager.tm_task_finish_callback(asr_task)
        
        
# 任务管理器
# 判断哪些任务能够运行
class Task_manager():
    class Run_task_return_code(Enum):
        cannot_run_cannot_next = 0      # 不能运行, 也不能执行下一个
        cannot_run_can_next    = auto() # 不能运行, 但可以执行下一个
        can_run_cannot_next    = auto() # 可以运行, 但不能执行下一个
        can_run_can_next       = auto() # 可以运行, 也可以执行下一个
    
    def __init__(self,robot=None):
        self.robot            = robot                 # 执行任务的机器人
        self.finished_tasks   = task.Task_sequence("finished_tasks")  # 已经完成的任务列表
        self.executed_tasks   = task.Task_sequence("executed_tasks")  # 正在执行的任务列表
        # self.conflicting_task = task.Task_sequence()  # 冲突的任务列表(可以并行, 但因为硬件冲突暂时无法并行)
        self.waiting_task     = task.Task_sequence("waiting_task")  # 等待执行的任务列表
        self.can_run_state    = True                  #是否能够执行任务
        # 每0.1s执行一次任务
        # TODO:调试时为3秒
        timer = rospy.Timer(rospy.Duration(0.3), self.timer_callback)
        # timer = rospy.Timer(rospy.Duration(system.anchor_point.time_interval_for_task), self.timer_callback)
    
    # 任务完成回调
    def tm_task_finish_callback(self, current_task:task.Task, status=None, result=None):
        rospy.loginfo(f"node: {rospy.get_name()}, task_manager, task : {current_task.task_index} call tm_task_finish_callback")
        # 让任务管理器恢复正常
        if current_task.parallel == task.Task.Task_parallel.NOTALLOWED:
            self.can_run_state = True
            
        if current_task.if_finished():
            rospy.loginfo(f"task index {current_task.task_index} is finished()")
            try:
                self.executed_tasks.remove_task(current_task) # 在执行的任务中移除
                self.finished_tasks.add(current_task)         # 添加到已完成的任务中
                rospy.loginfo(f"node: {rospy.get_name()}, task_manager, task index : {current_task.task_index} remove task from executed_tasks!")
            except Exception as e:
                rospy.loginfo(f"node: {rospy.get_name()}, task_manager, task index : {current_task.task_index} remove task from executed_tasks failed!!! \nError: {e}")
        else:
            rospy.loginfo(f"task  index {current_task.task_index} is not finish !!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            
        # 更新硬件状态
        if current_task.task_type.task_type.__class__ == task.Task_type.Task_navigate:
            system.robot.robot_status = robot.Robot.Robot_status.IDLE  # 机器人状态更新
        # 机械臂任务
        elif current_task.task_type.task_type.__class__ == task.Task_type.Task_manipulation:
            system.robot.update_arm_status(current_task.arm_id,robot.manipulation_status.arm.status.IDLE)
        # 图像识别任务
        elif current_task.task_type.task_type.__class__ == task.Task_type.Task_image_rec:
            system.robot.update_arm_status(current_task.camera_id,robot.manipulation_status.arm.status.IDLE)
            

    # 定时器任务
    @staticmethod
    def timer_callback(event):
        # rospy.loginfo("Task manager timer callback")
        index = 0
        while index < len(system.task_manager.waiting_task.task_list):
            current_task = system.task_manager.waiting_task.task_list[index]
            return_code = system.task_manager.task_can_run(current_task)
            # rospy.loginfo(f"task {current_task.task_index} return code is {return_code}")
            # 不能运行, 也不能下一个
            if return_code == Task_manager.Run_task_return_code.cannot_run_cannot_next:
                break
            # 不能运行, 但是能下一个
            elif return_code == Task_manager.Run_task_return_code.cannot_run_can_next:
                index += 1  # 下一个执行序号+1
                continue
            # 能运行
            else:
                # 将运行任务添加到正在执行的任务列表中
                system.task_manager.waiting_task.remove_task(current_task)
                system.task_manager.executed_tasks.add(current_task)
                
                # 导航任务
                if current_task.task_type.task_type.__class__ == task.Task_type.Task_navigate:
                    system.navigation_actuator.run(current_task)
                # 机械臂任务
                elif current_task.task_type.task_type.__class__ == task.Task_type.Task_manipulation:
                    system.manipulator_actuator.run(current_task)
                # 图像识别任务
                elif current_task.task_type.task_type.__class__ == task.Task_type.Task_image_rec:
                    system.image_rec_actuator.run(current_task)
                # 语音识别任务
                elif current_task.task_type.task_type.__class__ == task.Task_type.Task_speech_recognition:
                    system.asr_actuator.run(current_task)
                # 功能性暂停任务
                elif current_task.task_type.task_type == task.Task_type.Task_function.PAUSE:
                    # system.task_manager.waiting_task.remove_task(current_task)
                    # system.task_manager.executed_tasks.add(current_task)
                    current_task.update_start_status()
                    current_task.update_end_status()
                    system.task_manager.tm_task_finish_callback(current_task,None,None)
                    rospy.loginfo(f"node: {rospy.get_name()}, run a PAUSE task")
                    break
                
                # 能运行, 但是不能下一个
                if return_code == Task_manager.Run_task_return_code.can_run_cannot_next:
                    # 完成前, 不允许进行下一个任务
                    if current_task.task_type.task_type.__class__ == task.Task_type.Task_speech_recognition:
                        break
                    system.task_manager.can_run_state = False
                    break
                # 能运行, 也能下一个
                elif return_code == Task_manager.Run_task_return_code.can_run_can_next:
                    continue
                else:
                    raise ValueError("Invalid return code")
    
    # 判断任务是否能够运行
    def task_can_run(self,current_task:task.Task):
        # rospy.loginfo(f"robot status: {system.robot}")
        # 不能执行下一个任务
        if self.can_run_state == False:
            rospy.loginfo(f"node: {rospy.get_name()}, task index {current_task.task_index} can not run, because can_run_state is {self.can_run_state}")
            return Task_manager.Run_task_return_code.cannot_run_cannot_next
        
        # 任务不支持并行
        if current_task.parallel == task.Task.Task_parallel.NOTALLOWED:
            # 任务的前置任务是否完成
            if current_task.predecessor_tasks.has_been_done() == True:
                # 任务是否准备好
                if current_task.status == task.Task.Task_status.NOTREADY:
                    # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because it is not ready")
                    return Task_manager.Run_task_return_code.cannot_run_cannot_next
                elif current_task.status == task.Task.Task_status.BEREADY:
                    # 正在执行的任务是否为0
                    if self.executed_tasks.get_task_count() == 0:
                        return Task_manager.Run_task_return_code.can_run_cannot_next
                    else:
                        # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because other task is running")
                        return Task_manager.Run_task_return_code.cannot_run_cannot_next
                else:
                    raise ValueError("task.Task.Task_status error!!!!!!!")
            else:
                # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because predecessor task has not been done")
                return Task_manager.Run_task_return_code.cannot_run_cannot_next
        
        # 任务支持并行
        elif current_task.parallel == task.Task.Task_parallel.ALL:
            # 任务的前置任务是否完成
            if current_task.predecessor_tasks.has_been_done() == True:
                # 任务是否准备好
                if current_task.status == task.Task.Task_status.NOTREADY:
                    # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because it is not ready")
                    return Task_manager.Run_task_return_code.cannot_run_can_next
                elif current_task.status == task.Task.Task_status.BEREADY:
                    # 硬件资源是否支持运行
                    # 导航任务检测机器人是否在运动
                    if current_task.task_type.task_type.__class__ == task.Task_type.Task_navigate:
                        robot_status = system.robot.get_robot_status()
                        if robot_status == robot.Robot.Robot_status.MOVING:
                            # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because robot is moving")
                            return Task_manager.Run_task_return_code.cannot_run_can_next
                        elif robot_status == robot.Robot.Robot_status.IDLE:
                            # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can run. navigation task")
                            return Task_manager.Run_task_return_code.can_run_can_next
                        else:
                            raise ValueError("robot.Robot.Robot_status error!!!!!!!")
                    # 手臂任务检测手臂是否闲置
                    elif current_task.task_type.task_type.__class__ == task.Task_type.Task_manipulation:
                        # 如果是松开零食的任务, 则还需要判断松开点是否空闲
                        if current_task.task_type.task_type == task.Task_type.Task_manipulation.Lossen_snack:
                            if system.robot.lossen_snack_point_status == robot.Robot.Common_status.IDLE:
                                pass
                            elif system.robot.lossen_snack_point_status == robot.Robot.Common_status.BUSY:
                                return Task_manager.Run_task_return_code.cannot_run_can_next
                            
                        if system.robot.is_arm_idle(current_task.arm_id) == True:
                            # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can run. manipulation task")
                            return Task_manager.Run_task_return_code.can_run_can_next
                        else:
                            # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because arm {current_task.arm_id} is busy")
                            return Task_manager.Run_task_return_code.cannot_run_can_next
                    # 图像任务检测手臂是否闲置
                    elif current_task.task_type.task_type.__class__ == task.Task_type.Task_image_rec:
                        if system.robot.is_arm_idle(current_task.camera_id) == True:
                            # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can run. image_rec task")
                            return Task_manager.Run_task_return_code.can_run_can_next
                        else:
                            # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because arm {current_task.camera_id} is busy")
                            return Task_manager.Run_task_return_code.cannot_run_can_next
                    else:
                        raise ValueError(f"task.Task.Task_type {current_task.task_type.task_type.__class__} error!!!!!!!")
                else:
                    raise ValueError("task.Task.Task_status error!!!!!!!")
            else:
                # rospy.loginfo(f"node: {rospy.get_name()}, task {current_task.task_index} can not run, because predecessor task has not been done")
                return Task_manager.Run_task_return_code.cannot_run_can_next
        else :
            raise ValueError("task_can_run function error!!!!!!!")


# 订单驱动的任务安排
# 需要提供订单下单服务
class Order_driven_task_schedul():
    # instance = None
    def __init__(self,task_manager:Task_manager):
        self.task_manager = task_manager
        self.order_list = []
        self.server = rospy.Subscriber(utilis.Topic_name.make_order,msg.OrderInfo,self.do_order_req,self,queue_size=10)
    
    # 下单服务回调
    @staticmethod
    def do_order_req(order_info:msg.OrderInfo,obj:Order_driven_task_schedul):
        rospy.loginfo(f"node name {rospy.get_name()}, order driven task schedul, get new order")
        new_order = order.Order.instantialize_from_msg(order_info)
        log.log_new_order_info(new_order)
        obj.order_list.append(new_order)
        obj.update_task(new_order)
        
    def add_asr_task(self):
        self.task_manager.waiting_task.add(self.create_task_speech_recognition())
    
    # 根据订单更新任务
    def update_task(self,order:order.Order):
        result = False
        # 增
        if order.operation == order.Operation.ADD:
            rospy.loginfo(f"node name {rospy.get_name()}, order driven task schedul, add new order")
            rospy.loginfo(f"order info \r\n{order}")
            result = self.add_task(order)
        # 删
        elif order.operation == order.Operation.DELETE:
            result = self.delete_task(order)
        # 改
        elif order.operation == order.Operation.MODIFY:
            pass
        # 查
        elif order.operation == order.Operation.CHECK:
            pass
        else:
            rospy.logerr(f"node name {rospy.get_name()}, order driven task schedul, invalid operation")
        return result
    
    # 新增任务
    def add_task(self,order:order.Order): 
        new_task_sequence = task.Task_sequence()
        # 有零食请求
        if order.has_snack_request():
            new_task_sequence.add(self.create_tasks_grasp_snack(order.snack_list,order.table_id))

        # 有饮料请求
        if order.has_drink_request():
            new_task_sequence.add(self.create_tasks_get_drink(order.table_id))

        # 更新组ID
        new_task_sequence.update_group_id(order.order_id)
        
        # # 添加返回起始点的任务
        # new_task_sequence.add(self.create_task_navigate_to_init_point())
        
        # # 添加到任务管理器待执行队列, 并删除最后的返回起始点 
        # self.task_manager.waiting_task.add(new_task_sequence,del_last_naviagte_to_init_point = True)
        
        # 添加语音识别的任务
        new_task_sequence.add(self.create_task_speech_recognition())
        
        # 添加到任务管理器待执行队列, 删除最后的语音识别任务
        self.task_manager.waiting_task.add_and_del_last_asr_task(new_task_sequence,del_last_asr_task = True)
        
        return new_task_sequence
        
    # 删除任务
    def delete_task(self,task):
        pass
        
    #  拿零食
    def create_tasks_grasp_snack(self,snack_list:order.Snack_list,table_id:utilis.Device_id):
        tasks_pick_snack        = task.Task_sequence()
        
        #  手臂移到空闲位, 并关闭夹爪
        task_left_arm_idle = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE, None, utilis.Device_id.LEFT, \
            system.anchor_point.left_arm_idle,arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="left arm move to idle prepare for pick snack")
        task_left_arm_idle.parallel = task.Task.Task_parallel.ALL  # 可并行
        tasks_pick_snack.add(task_left_arm_idle)
        
        #  手臂移到空闲位, 并关闭夹爪
        task_right_arm_idle = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE, None, utilis.Device_id.RIGHT, \
            system.anchor_point.right_arm_idle,arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm move to idle prepare for pick snack")
        task_right_arm_idle.parallel = task.Task.Task_parallel.ALL  # 可并行
        tasks_pick_snack.add(task_right_arm_idle)
        
        #  前往零食桌
        task_navigation_to_snack_desk  = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_snack_desk, None, \
            system.anchor_point.map_snack_desk, name="navigation to snack desk")
        
        # 如果不是调试模式, 则可以并行
        if not DEBUG_NAVIGATION:
            task_navigation_to_snack_desk.parallel = task.Task.Task_parallel.ALL
        tasks_pick_snack.add(task_navigation_to_snack_desk)

        # 向前移动到零食桌
        task_navigation_move_foward_to_snack_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_forward, None,\
            system.anchor_point.snack_deck_move_forward_pose, name="navigation move forward to snack desk")
        task_navigation_move_foward_to_snack_desk.add_predecessor_task(task_navigation_to_snack_desk)           # 前置任务, 导航到零食桌前20cm
        task_navigation_move_foward_to_snack_desk.parallel = task.Task.Task_parallel.ALL                        # 可并行
        task_navigation_move_foward_to_snack_desk.set_move_back_second(system.anchor_point.snack_deck_move_forward_pose.run_time)  # 移动后退1s
        tasks_pick_snack.add(task_navigation_move_foward_to_snack_desk)
        
        #  将左臂抬到指定位置(食物框识别位置)
        task_left_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.LEFT, \
            system.anchor_point.left_arm_container_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="left arm move to rec container")
        task_left_arm_to_rec_contianer.parallel = task.Task.Task_parallel.ALL             # 可并行
        task_left_arm_to_rec_contianer.add_predecessor_task(task_navigation_move_foward_to_snack_desk)    # 前置任务, 机器人移动到位
        tasks_pick_snack.add(task_left_arm_to_rec_contianer)
        
        #  将右臂抬到指定位置(食物框识别位置)
        task_right_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_container_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm move to rec container")
        task_right_arm_to_rec_contianer.parallel = task.Task.Task_parallel.ALL           # 可并行
        task_right_arm_to_rec_contianer.add_predecessor_task(task_navigation_move_foward_to_snack_desk)  # 前置任务, 机器人移动到位
        tasks_pick_snack.add(task_right_arm_to_rec_contianer)
        
        #  左摄像头食物框识别(可前后并行，固定)
        task_left_camera_rec_container = task.Task_image_rec(task.Task_type.Task_image_rec.CONTAINER, None, utilis.Device_id.LEFT,\
            name="left camera rec container")
        task_left_camera_rec_container.parallel = task.Task.Task_parallel.ALL               # 可并行
        task_left_camera_rec_container.set_sleep_time_before_task(system.anchor_point.time_before_get_image) # 获取照片前暂停一下，以免图片模糊
        task_left_camera_rec_container.add_predecessor_task(task_left_arm_to_rec_contianer) # 前置任务, 左臂移动到食物框识别位置
        task_left_camera_rec_container.add_predecessor_task(task_navigation_to_snack_desk)  # 前置任务, 导航到零食桌
        tasks_pick_snack.add(task_left_camera_rec_container)
        
        #  右摄像头食物框识别(可前后并行，固定)
        task_right_camera_rec_container = task.Task_image_rec(task.Task_type.Task_image_rec.CONTAINER, None, utilis.Device_id.RIGHT,\
            name="right camera rec container")
        task_right_camera_rec_container.parallel = task.Task.Task_parallel.ALL                  # 可并行
        task_right_camera_rec_container.set_sleep_time_before_task(system.anchor_point.time_before_get_image) # 获取照片前暂停一下，以免图片模糊
        task_right_camera_rec_container.add_predecessor_task(task_right_arm_to_rec_contianer)   # 前置任务, 右臂移动到食物框识别位置
        task_right_camera_rec_container.add_predecessor_task(task_navigation_to_snack_desk)     # 前置任务, 导航到零食桌
        
        tasks_pick_snack.add(task_right_camera_rec_container)
        
        #  将左臂抬到零食识别位置(可前后并行，固定)
        task_left_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.LEFT, \
            system.anchor_point.left_arm_snack_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="left arm move to rec snack")
        task_left_arm_to_rec_snack.parallel = task.Task.Task_parallel.ALL                     # 可并行
        task_left_arm_to_rec_snack.add_predecessor_task(task_left_camera_rec_container)       # 前置任务, 左摄像头食物框识别
        tasks_pick_snack.add(task_left_arm_to_rec_snack)
        
        #  将右臂抬到零食识别位置(可前后并行，固定)
        task_right_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.RIGHT, \
            system.anchor_point.right_arm_snack_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm move to rec snack")
        task_right_arm_to_rec_snack.parallel = task.Task.Task_parallel.ALL                    # 可并行
        task_right_arm_to_rec_snack.add_predecessor_task(task_right_camera_rec_container)     # 前置任务, 右摄像头食物框识别
        tasks_pick_snack.add(task_right_arm_to_rec_snack)
        
        #  左、右摄像头零食识别(不可并行，动态)
        task_rec_snack = task.Task_image_rec(task.Task_type.Task_image_rec.SNACK, None, utilis.Device_id.LEFT_RIGHT,\
            name="two arms rec snack")
        task_rec_snack.set_snack_list(snack_list)
        task_rec_snack.set_sleep_time_before_task(system.anchor_point.time_before_get_image)   # 获取照片前暂停一下，以免图片模糊
        task_rec_snack.add_predecessor_task(task_left_arm_to_rec_snack)                        # 前置任务, 左臂移动到零食识别位置
        task_rec_snack.add_predecessor_task(task_right_arm_to_rec_snack)                       # 前置任务, 右臂移动到零食识别位置
        tasks_pick_snack.add(task_rec_snack)
        
        # 左臂夹取零食框前的躲避动作
        task_left_arm_grap_container_dodge = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_dodge,None,utilis.Device_id.LEFT,\
                [copy.deepcopy(system.anchor_point.left_arm_container_grip_dodge)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="left arm grap container dodge")
        task_left_arm_grap_container_dodge.parallel = task.Task.Task_parallel.ALL          # 可并行
        task_left_arm_grap_container_dodge.status   = task.Task.Task_status.BEREADY        
        tasks_pick_snack.add(task_left_arm_grap_container_dodge)
        
        task_left_camera_rec_container.add_need_modify_task(task_left_arm_grap_container_dodge)  # 识别容器修改
        
        # 右臂夹取零食框前的躲避动作
        task_right_arm_grap_container_dodge = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_dodge,None,utilis.Device_id.RIGHT,\
                [copy.deepcopy(system.anchor_point.right_arm_container_grip_dodge)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="right arm grap container dodge")
        task_right_arm_grap_container_dodge.parallel = task.Task.Task_parallel.ALL          # 可并行
        task_right_arm_grap_container_dodge.status   = task.Task.Task_status.BEREADY        
        tasks_pick_snack.add(task_right_arm_grap_container_dodge)
        
        task_right_camera_rec_container.add_need_modify_task(task_right_arm_grap_container_dodge)  # 识别容器修改
        

        # 零食抓取任务
        snack_count = snack_list.get_all_snack_count()
        for i in range(snack_count):
            tasks_pick_snack.add(self.create_task_grasp_snack(task_rec_snack,task_left_camera_rec_container,task_right_camera_rec_container,\
                task_left_arm_grap_container_dodge,task_right_arm_grap_container_dodge))
        
        # 左臂夹取零食框, 准备动作
        task_left_arm_grap_container_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_pre,None,utilis.Device_id.LEFT,\
                [copy.deepcopy(system.anchor_point.left_arm_container_grip_pre)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="left arm grap container prepare")
        task_left_arm_grap_container_pre.parallel = task.Task.Task_parallel.ALL          # 可并行
        task_left_arm_grap_container_pre.status   = task.Task.Task_status.NOTREADY       # 需要参数
        task_left_arm_grap_container_pre.add_predecessor_task(task_left_arm_grap_container_dodge)   # 前置任务, 躲避动作
        task_left_arm_grap_container_pre.add_predecessor_task(task_right_arm_grap_container_dodge)  # 前置任务, 躲避动作
        tasks_pick_snack.add(task_left_arm_grap_container_pre)
        task_left_camera_rec_container.add_need_modify_task(task_left_arm_grap_container_pre)  # 识别容器绑定 夹取容器准备动作
        
        # 右臂夹取零食框, 准备动作
        task_right_arm_grap_container_pre    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_pre,None,utilis.Device_id.RIGHT,\
                [copy.deepcopy(system.anchor_point.right_arm_container_grip_pre)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="right arm grap container prepare")
        task_right_arm_grap_container_pre.parallel = task.Task.Task_parallel.ALL          # 可并行  
        task_right_arm_grap_container_pre.status   = task.Task.Task_status.NOTREADY       # 需要参数 
        task_right_arm_grap_container_pre.add_predecessor_task(task_left_arm_grap_container_dodge)   # 前置任务, 躲避动作
        task_right_arm_grap_container_pre.add_predecessor_task(task_right_arm_grap_container_dodge)  # 前置任务, 躲避动作
        tasks_pick_snack.add(task_right_arm_grap_container_pre)
        task_right_camera_rec_container.add_need_modify_task(task_right_arm_grap_container_pre) # 识别容器绑定 夹取容器
        
        
        # 功能暂停任务(等待全部零食抓取完毕)
        task_function_pause = task.Task_function(task.Task_type.Task_function.PAUSE, None,name="function pause")
        task_function_pause.add_predecessor_task(task_left_arm_grap_container_pre)  
        task_function_pause.add_predecessor_task(task_right_arm_grap_container_pre)  
        tasks_pick_snack.add(task_function_pause)
        
        # 左臂夹取零食框
        task_left_arm_grap_container    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container,None,utilis.Device_id.LEFT,\
                [copy.deepcopy(system.anchor_point.left_arm_container_grip)],\
                    [arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                        name="left arm grap container")
        task_left_arm_grap_container.parallel = task.Task.Task_parallel.ALL                  # 可并行
        task_left_arm_grap_container.add_predecessor_task(task_left_arm_grap_container_pre)  # 前置任务, 完成准备动作
        tasks_pick_snack.add(task_left_arm_grap_container)
            
        
        # 右臂夹取零食框
        task_right_arm_grap_container    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container,None,utilis.Device_id.RIGHT,\
                [copy.deepcopy(system.anchor_point.right_arm_container_grip)],\
                    [arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                        name="right arm grap container")
        task_right_arm_grap_container.parallel = task.Task.Task_parallel.ALL                   # 可并行  
        task_right_arm_grap_container.add_predecessor_task(task_right_arm_grap_container_pre)  # 前置任务, 完成准备动作
        tasks_pick_snack.add(task_right_arm_grap_container)
        
        
        #  左、右臂将零食框放到指定高度
        task_arm_dilivery_container   = task.Task_manipulation(task.Task_type.Task_manipulation.Deliever_container,None,utilis.Device_id.LEFT_RIGHT,\
                [system.anchor_point.left_arm_container_delivery,system.anchor_point.right_arm_container_delivery],\
                    [arm.GripMethod.DONTCANGE,arm.GripMethod.DONTCANGE], arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                        name="two arms dilivery container")
        
        task_arm_dilivery_container.parallel = task.Task.Task_parallel.ALL
        task_arm_dilivery_container.set_subtask_count(2)  # 两个子任务
        task_arm_dilivery_container.add_predecessor_task(task_left_arm_grap_container)        # 前置任务, 左臂抓取容器
        task_arm_dilivery_container.add_predecessor_task(task_right_arm_grap_container)       # 前置任务, 右臂抓取容器
        tasks_pick_snack.add(task_arm_dilivery_container)
        
        # 机器人后退
        task_move_back_from_snack_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,\
            system.anchor_point.snack_deck_move_back_pose, name="move back from snack desk")
        task_move_back_from_snack_desk.set_move_back_second(system.anchor_point.snack_deck_move_back_pose.run_time)  # 设置运行时间
        task_move_back_from_snack_desk.add_predecessor_task(task_right_arm_grap_container)   # 前置任务, 左臂抓取容器
        task_move_back_from_snack_desk.add_predecessor_task(task_left_arm_grap_container)    # 前置任务, 右臂抓取容器
        task_move_back_from_snack_desk.parallel = task.Task.Task_parallel.ALL
        tasks_pick_snack.add(task_move_back_from_snack_desk)

        #  导航前往n号桌
        if table_id == utilis.Device_id.LEFT.value:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_left_service_desk,None,\
                system.anchor_point.map_left_service_desk, name = "navigation to left service desk")
            
            # 向前移动到左边服务桌子
            task_navigation_move_foward_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_forward, None,\
                system.anchor_point.left_deck_move_forward_pose, name="navigation move forward to left desk")
            task_navigation_move_foward_to_service_desk.add_predecessor_task(task_navigation_to_service_desk)         # 前置任务, 导航到服务桌前20cm
            task_navigation_move_foward_to_service_desk.parallel = task.Task.Task_parallel.ALL                        # 可并行
            task_navigation_move_foward_to_service_desk.set_move_back_second(system.anchor_point.left_deck_move_forward_pose.run_time)  # 设置运行时间
        
        
        elif table_id == utilis.Device_id.RIGHT.value:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_right_service_desk,None,\
                system.anchor_point.map_right_service_desk, name="navigation to right service desk")
            
            # 向前移动到右边服务桌子
            task_navigation_move_foward_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_forward, None,\
                system.anchor_point.right_deck_move_forward_pose, name="navigation move forward to right desk")
            task_navigation_move_foward_to_service_desk.add_predecessor_task(task_navigation_to_service_desk)         # 前置任务, 导航到服务桌前20cm
            task_navigation_move_foward_to_service_desk.parallel = task.Task.Task_parallel.ALL                        # 可并行
            task_navigation_move_foward_to_service_desk.set_move_back_second(system.anchor_point.right_deck_move_forward_pose.run_time)  # 设置运行时间
            
        else:
            raise ValueError("Invalid table_id")
        
        # 不调试则可以并行
        if not DEBUG_NAVIGATION:
            task_navigation_to_service_desk.parallel = task.Task.Task_parallel.ALL           # 可并行
        task_navigation_to_service_desk.add_predecessor_task(task_move_back_from_snack_desk) # 前置任务, 机器人后退完成
        
        tasks_pick_snack.add(task_navigation_to_service_desk)                                # 添加任务
        tasks_pick_snack.add(task_navigation_move_foward_to_service_desk)                    # 添加任务
        
        
        #  将左、右臂放到指定位置后，松开(不可并行)
        task_arm_placement_container   = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_container,None,utilis.Device_id.LEFT_RIGHT,\
                [system.anchor_point.left_arm_container_placement,system.anchor_point.right_arm_container_placement],\
                [arm.GripMethod.OPEN,arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                    name="two arms placement container")
        task_arm_placement_container.add_predecessor_task(task_navigation_move_foward_to_service_desk)  # 前置任务, 到服务桌前了
        task_arm_placement_container.set_subtask_count(2)
        tasks_pick_snack.add(task_arm_placement_container)
    
        #  将左,右臂放到空闲位置
        task_arms_idle   = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE,None,utilis.Device_id.LEFT_RIGHT,\
                [system.anchor_point.left_arm_idle,system.anchor_point.right_arm_idle],\
                [arm.GripMethod.CLOSE,arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.XYZ,
                name="two arms move to idle")
        task_arms_idle.set_subtask_count(2)                                # 两个子任务
        task_arms_idle.parallel = task.Task.Task_parallel.ALL              # 可并行
        task_arms_idle.add_predecessor_task(task_arm_placement_container)  # 前置任务, 完成放置容器
        tasks_pick_snack.add(task_arms_idle)
        
        # 机器人后退
        if table_id == utilis.Device_id.LEFT.value:
            task_move_back_from_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,\
            system.anchor_point.left_deck_move_back_pose, name="move back from left service desk")
            task_move_back_from_service_desk.set_move_back_second(system.anchor_point.left_deck_move_back_pose.run_time)    # 设置运行时间
        elif table_id == utilis.Device_id.RIGHT.value:
            task_move_back_from_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,\
            system.anchor_point.right_deck_move_back_pose, name="move back from right service desk")
            task_move_back_from_service_desk.set_move_back_second(system.anchor_point.right_deck_move_back_pose.run_time)    # 设置运行时间
        else:
            raise ValueError("Invalid table_id")
        task_move_back_from_service_desk.parallel = task.Task.Task_parallel.ALL              # 可并行
        task_move_back_from_service_desk.add_predecessor_task(task_arm_placement_container)  # 前置任务, 完成放置容器
        tasks_pick_snack.add(task_move_back_from_service_desk)
        
        # 功能性暂停(等待前面的动作全部完成)
        task_function_pause2 = task.Task_function(task.Task_type.Task_function.PAUSE,None,name="function pause")
        tasks_pick_snack.add(task_function_pause2)
        
        return tasks_pick_snack
    
    # 拿饮料
    def create_tasks_get_drink(self, table_id:utilis.Device_id):
        tasks_get_drink = task.Task_sequence()  
        # 导航前往饮料桌
        task_navigation_to_drink_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_drink_desk,None,\
            system.anchor_point.map_drink_desk, name="navigation to drink desk")
        
        # 非调试模式, 可并行
        if not DEBUG_NAVIGATION:
            task_navigation_to_drink_desk.parallel = task.Task.Task_parallel.ALL # 可并行
        tasks_get_drink.add(task_navigation_to_drink_desk)
        
        # 向前移动到饮料桌子
        task_navigation_move_foward_to_drink_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_forward, None,\
            system.anchor_point.drink_deck_move_forward_pose, name="navigation move forward to drink desk")
        task_navigation_move_foward_to_drink_desk.add_predecessor_task(task_navigation_to_drink_desk)                               # 前置任务, 导航到服务桌前20cm
        task_navigation_move_foward_to_drink_desk.parallel = task.Task.Task_parallel.ALL                                            # 可并行
        task_navigation_move_foward_to_drink_desk.set_move_back_second(system.anchor_point.drink_deck_move_forward_pose.run_time)   # 移动前进2s
        task_navigation_move_foward_to_drink_desk.set_sleep_time_after_task(0.5)                                                    # 暂停2s,给人挪机器人的时间
        tasks_get_drink.add(task_navigation_move_foward_to_drink_desk)
        
        #  左臂抬到指定位置识别咖啡机开关 开
        task_left_arm_to_rec_coffee_machine_turn_on = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_machine_switch, None, utilis.Device_id.LEFT, \
            system.anchor_point.left_arm_machine_turn_on_rec, arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="left arm move to rec coffee machine turn on")
        task_left_arm_to_rec_coffee_machine_turn_on.parallel = task.Task.Task_parallel.ALL         # 可并行
        tasks_get_drink.add(task_left_arm_to_rec_coffee_machine_turn_on)                           
        
        #  右臂抬到指定位置  进行准备 ，识别杯子和机器(中间点)
        task_right_arm_to_rec_cup_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_cup_machine, None, utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_rec_pre, arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm move to rec cup machine prepare")
        task_right_arm_to_rec_cup_pre.parallel = task.Task.Task_parallel.ALL                       # 可并行
        tasks_get_drink.add(task_right_arm_to_rec_cup_pre)
        
        #  右臂抬到指定位置识别杯子和机器
        task_right_arm_to_rec_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_cup_machine, None, utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm move to rec cup machine")
        task_right_arm_to_rec_cup.parallel = task.Task.Task_parallel.ALL                           # 可并行
        task_right_arm_to_rec_cup.add_predecessor_task(task_right_arm_to_rec_cup_pre)              # 前置任务, 右臂到达了中间点
        tasks_get_drink.add(task_right_arm_to_rec_cup)
        
        #  左臂图像识别(咖啡机开关位置, 开的时候)
        task_left_camera_rec_coffee_machine_turn_on = task.Task_image_rec(task.Task_type.Task_image_rec.COFFEE_MACHINE_SWITCH_ON,None,utilis.Device_id.LEFT,\
            name="left camera rec coffee machine turn on")
        task_left_camera_rec_coffee_machine_turn_on.parallel = task.Task.Task_parallel.ALL                               # 可并行
        task_left_camera_rec_coffee_machine_turn_on.set_sleep_time_before_task(system.anchor_point.time_before_get_image) # 获取照片前暂停一下，以免图片模糊
        task_left_camera_rec_coffee_machine_turn_on.add_predecessor_task(task_left_arm_to_rec_coffee_machine_turn_on)    # 前置任务, 左臂到位
        task_left_camera_rec_coffee_machine_turn_on.add_predecessor_task(task_navigation_move_foward_to_drink_desk)                  # 前置任务, 导航到饮料桌
        tasks_get_drink.add(task_left_camera_rec_coffee_machine_turn_on)
        
        #  右摄像头图像识别(杯子位置、咖啡机位置)(可并行，固定)
        task_right_camera_rec_cup_machine = task.Task_image_rec(task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE,None,utilis.Device_id.RIGHT,\
            name="right camera rec cup machine")
        task_right_camera_rec_cup_machine.parallel = task.Task.Task_parallel.ALL                               # 可并行
        task_right_camera_rec_cup_machine.set_sleep_time_before_task(system.anchor_point.time_before_get_image)# 获取照片前暂停一下，以免图片模糊
        task_right_camera_rec_cup_machine.add_predecessor_task(task_right_arm_to_rec_cup)                      # 前置任务, 右臂到位,
        task_right_camera_rec_cup_machine.add_predecessor_task(task_navigation_move_foward_to_drink_desk)      # 前置任务, 导航到饮料桌
        tasks_get_drink.add(task_right_camera_rec_cup_machine)
        
        #  右臂夹取杯子准备动作(中间点)
        task_right_arm_grasp_cup_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_cup_pre,None,utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_grab_pre,arm.GripMethod.OPEN, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm grasp cup prepare")
        task_right_arm_grasp_cup_pre.parallel = task.Task.Task_parallel.ALL     # 可并行
        task_right_arm_grasp_cup_pre.status   = task.Task.Task_status.NOTREADY  # 需要参数
        tasks_get_drink.add(task_right_arm_grasp_cup_pre)
        task_right_camera_rec_cup_machine.add_need_modify_task(task_right_arm_grasp_cup_pre)  # 右臂绑定 夹取杯子和移动杯子
        
        #  右臂夹取杯子
        task_right_arm_grasp_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_cup,None,utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_grab,arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm grasp cup")
        task_right_arm_grasp_cup.parallel = task.Task.Task_parallel.ALL                      # 可并行
        task_right_arm_grasp_cup.status   = task.Task.Task_status.NOTREADY                   # 需要参数
        task_right_arm_grasp_cup.add_predecessor_task(task_right_arm_grasp_cup_pre)          # 前置任务, 到达中间点
        tasks_get_drink.add(task_right_arm_grasp_cup)
        task_right_camera_rec_cup_machine.add_need_modify_task(task_right_arm_grasp_cup)     # 右臂绑定 夹取杯子和移动杯子

        # 左臂放置到按钮下方, 进行准备
        task_left_arm_turn_on_machine_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_on_coffee_machine,None,utilis.Device_id.LEFT,\
            copy.deepcopy(system.anchor_point.left_arm_machine_turn_off_pre),arm.GripMethod.CLOSE,\
                arm_move_method = arm.ArmMoveMethod.Z_XY,\
                    name="left arm turn on machine prepare")
        task_left_arm_turn_on_machine_pre.parallel = task.Task.Task_parallel.ALL            # 可并行
        task_left_arm_turn_on_machine_pre.status   = task.Task.Task_status.NOTREADY         # 需要参数
        tasks_get_drink.add(task_left_arm_turn_on_machine_pre)
        task_left_camera_rec_coffee_machine_turn_on.add_need_modify_task(task_left_arm_turn_on_machine_pre)  # 绑定左臂识别任务
        
        # 左臂开启咖啡机, 向上拨一拨
        task_left_arm_turn_on_machine_click = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_on_coffee_machine,None,utilis.Device_id.LEFT,\
            copy.deepcopy(system.anchor_point.left_arm_machine_turn_on_click), arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                name="left arm turn on machine click!!!")
        task_left_arm_turn_on_machine_click.parallel = task.Task.Task_parallel.ALL                   # 可并行
        task_left_arm_turn_on_machine_click.status   = task.Task.Task_status.BEREADY                 
        task_left_arm_turn_on_machine_click.add_predecessor_task(task_left_arm_turn_on_machine_pre)  # 前置任务, 左臂到达开关下方
        task_left_arm_turn_on_machine_click.set_sleep_time_after_task(system.anchor_point.time_wait_for_turn_on_machine)   # 等待
        tasks_get_drink.add(task_left_arm_turn_on_machine_click)
        
        #  右臂将杯子挪到咖啡机
        task_right_arm_water_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Water_cup,None,utilis.Device_id.RIGHT,\
            copy.deepcopy(system.anchor_point.right_arm_cup_water), arm_move_method = arm.ArmMoveMethod.X_YZ,\
                name="right arm water cup")
        task_right_arm_water_cup.parallel = task.Task.Task_parallel.ALL                  # 可并行
        task_right_arm_water_cup.status   = task.Task.Task_status.NOTREADY
        task_right_arm_water_cup.add_predecessor_task(task_right_arm_grasp_cup)          # 前置任务, 要先抓到杯子  
        tasks_get_drink.add(task_right_arm_water_cup)
        # 修改为左臂识别任务
        task_left_camera_rec_coffee_machine_turn_on.add_need_modify_task(task_right_arm_water_cup) # 绑定右臂识别任务
        
        
        # 左臂转移动作
        task_left_arm_turn_tran_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_tra_coffee_machine,None,utilis.Device_id.LEFT,\
            copy.deepcopy(system.anchor_point.left_arm_machine_transfrom), arm_move_method = arm.ArmMoveMethod.XY_Z,\
                name="left arm trans to turn off machine")
        task_left_arm_turn_tran_machine.status   = task.Task.Task_status.NOTREADY                          # 需要参数
        task_left_arm_turn_tran_machine.parallel = task.Task.Task_parallel.ALL                             # 可并行
        task_left_arm_turn_tran_machine.add_predecessor_task(task_left_arm_turn_on_machine_click)
        tasks_get_drink.add(task_left_arm_turn_tran_machine)
        task_left_camera_rec_coffee_machine_turn_on.add_need_modify_task(task_left_arm_turn_tran_machine) # 绑定左臂识别任务绑定左臂识别任务
        
        
        # 左臂关闭咖啡机(!!!!!!!!!!!!!不可并行)
        task_left_arm_turn_off_machine_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_off_coffee_machine,None,utilis.Device_id.LEFT,\
            copy.deepcopy(system.anchor_point.left_arm_machine_turn_off_pre), arm_move_method = arm.ArmMoveMethod.Z_XY,\
                name="left arm turn off machine pre")
        task_left_arm_turn_off_machine_pre.status   = task.Task.Task_status.NOTREADY                          # 需要参数
        task_left_arm_turn_off_machine_pre.parallel = task.Task.Task_parallel.ALL                             # 可并行
        task_left_arm_turn_off_machine_pre.add_predecessor_task(task_left_arm_turn_tran_machine)
        tasks_get_drink.add(task_left_arm_turn_off_machine_pre)
        task_left_camera_rec_coffee_machine_turn_on.add_need_modify_task(task_left_arm_turn_off_machine_pre)  # 绑定左臂识别任务绑定左臂识别任务
        
        # 左臂关闭咖啡机(!!!!!!!!!!!!!不可并行), 向下拨一拨
        task_left_arm_turn_off_machine_click = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_off_coffee_machine,None,utilis.Device_id.LEFT,\
            copy.deepcopy(system.anchor_point.left_arm_machine_turn_off_click), arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                name="left arm turn off machine click!!!")
        task_left_arm_turn_off_machine_click.status   = task.Task.Task_status.BEREADY                 
        task_left_arm_turn_off_machine_click.add_predecessor_task(task_left_arm_turn_off_machine_pre)  # 前置任务,抓具放在了开关上面
        task_left_arm_turn_off_machine_click.set_sleep_time_after_task(system.anchor_point.time_wait_for_turn_off_machine)      # 等待3s
        tasks_get_drink.add(task_left_arm_turn_off_machine_click)
        
        #  饮料桌机器人后退
        task_move_back_from_drink_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,\
            system.anchor_point.drink_deck_move_back_pose, name="move back from drink desk")
        task_move_back_from_drink_desk.parallel = task.Task.Task_parallel.ALL                          # 可并行
        task_move_back_from_drink_desk.add_predecessor_task(task_left_arm_turn_off_machine_pre)        # 前置任务,左臂已经关闭咖啡机了
        task_move_back_from_drink_desk.set_move_back_second(system.anchor_point.drink_deck_move_back_pose.run_time) # 设置运行时间
        tasks_get_drink.add(task_move_back_from_drink_desk)
        
        #  左臂抬到休闲位
        task_left_arm_idle = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE,None,utilis.Device_id.LEFT,\
            system.anchor_point.left_arm_idle,arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="left arm move to idle")
        task_left_arm_idle.add_predecessor_task(task_move_back_from_drink_desk)                         # 前置任务, 机器人退后了
        task_left_arm_idle.parallel = task.Task.Task_parallel.ALL                                       # 可并行
        tasks_get_drink.add(task_left_arm_idle)
        
        #  将右臂(拿水)抬到指定位置
        task_right_arm_water_delivery = task.Task_manipulation(task.Task_type.Task_manipulation.Deliever_cup,None,utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_delivery,arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm water delivery")
        task_right_arm_water_delivery.parallel = task.Task.Task_parallel.ALL                # 可并行
        task_right_arm_water_delivery.add_predecessor_task(task_move_back_from_drink_desk)  # 前置任务, 机器人退后了
        tasks_get_drink.add(task_right_arm_water_delivery)
        
        
        #  导航前往n号桌
        if table_id == utilis.Device_id.LEFT.value:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_left_service_desk,None,\
                system.anchor_point.map_left_service_desk, name = "navigation to left service desk")
            
            # 向前移动到左边服务桌子
            task_navigation_move_foward_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_forward, None,\
                system.anchor_point.left_deck_move_forward_pose, name="navigation move forward to left desk")
            task_navigation_move_foward_to_service_desk.add_predecessor_task(task_navigation_to_service_desk)         # 前置任务, 导航到服务桌前20cm
            task_navigation_move_foward_to_service_desk.parallel = task.Task.Task_parallel.ALL                        # 可并行
            task_navigation_move_foward_to_service_desk.set_move_back_second(system.anchor_point.left_deck_move_forward_pose.run_time)    # 移动后退2s
        
        
        elif table_id == utilis.Device_id.RIGHT.value:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_right_service_desk,None,\
                system.anchor_point.map_right_service_desk, name="navigation to right service desk")
            
            # 向前移动到右边服务桌子
            task_navigation_move_foward_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_forward, None,\
                system.anchor_point.right_deck_move_forward_pose, name="navigation move forward to right desk")
            task_navigation_move_foward_to_service_desk.add_predecessor_task(task_navigation_to_service_desk)         # 前置任务, 导航到服务桌前20cm
            task_navigation_move_foward_to_service_desk.parallel = task.Task.Task_parallel.ALL                        # 可并行
            task_navigation_move_foward_to_service_desk.set_move_back_second(system.anchor_point.right_deck_move_forward_pose.run_time)    # 移动后退2s
            
        else:
            raise ValueError("Invalid table_id")
        
        # 非调试模式, 可并行
        if not DEBUG_NAVIGATION:
            task_navigation_to_service_desk.parallel = task.Task.Task_parallel.ALL                   # 可并行
        task_navigation_to_service_desk.add_predecessor_task(task_move_back_from_drink_desk)         # 前置任务, 机器人退后了
        tasks_get_drink.add(task_navigation_to_service_desk)                                         # 添加任务
        tasks_get_drink.add(task_navigation_move_foward_to_service_desk)                             # 添加任务

        #  将饮料臂放到指定位置后松开(不可并行)
        task_right_arm_placement_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_cup,None,utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_placement,arm.GripMethod.OPEN, arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
            name="right arm placement cup")
        task_right_arm_placement_cup.add_predecessor_task(task_navigation_move_foward_to_service_desk)  # 前置任务, 到达指定位置
        tasks_get_drink.add(task_right_arm_placement_cup)
        
        #  机器人后退
        if table_id == utilis.Device_id.LEFT.value:
            task_move_back_from_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,\
            system.anchor_point.left_deck_move_back_pose, name="move back from left service desk")
            task_move_back_from_service_desk.set_move_back_second(system.anchor_point.left_deck_move_back_pose.run_time)    # 设置运行时间
        elif table_id == utilis.Device_id.RIGHT.value:
            task_move_back_from_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,\
            system.anchor_point.right_deck_move_back_pose, name="move back from right service desk")
            task_move_back_from_service_desk.set_move_back_second(system.anchor_point.right_deck_move_back_pose.run_time)    # 设置运行时间
        else:
            raise ValueError("Invalid table_id")
        
        task_move_back_from_service_desk.parallel = task.Task.Task_parallel.ALL                    # 可并行
        task_move_back_from_service_desk.add_predecessor_task(task_right_arm_placement_cup)        # 前置任务, 右臂放好了杯子
        tasks_get_drink.add(task_move_back_from_service_desk)
        
        #  将左,右臂放到空闲位置(可并行)
        task_arms_idle   = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE,None,utilis.Device_id.LEFT_RIGHT,\
                [system.anchor_point.left_arm_idle,system.anchor_point.right_arm_idle],\
                [arm.GripMethod.CLOSE,arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.XYZ,\
                    name="two arms move to idle")
        task_arms_idle.add_predecessor_task(task_move_back_from_service_desk)                     # 前置任务,离开桌子,才进行后续操作
        task_arms_idle.set_subtask_count(2)                                                       # 两个子任务
        task_arms_idle.parallel = task.Task.Task_parallel.ALL                                     # 可并行
        tasks_get_drink.add(task_arms_idle)
        
        # 功能性暂停(等待前面的动作全部完成)
        task_function_pause = task.Task_function(task.Task_type.Task_function.PAUSE,None,name="function pause")
        tasks_get_drink.add(task_function_pause)
        
        # 赋值
        return tasks_get_drink

    # 创建前往初始点的任务
    def create_task_navigate_to_init_point(self):
        #  前往初始位置
        init_task = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,None,system.anchor_point.map_initial_pose,\
            name="navigate to initial point")
        init_task.parallel = task.Task.Task_parallel.ALL
        return init_task
    
    # 创建语音识别任务
    def create_task_speech_recognition(self):
        asr_task = task.Task_speech_recognition(task.Task_type.Task_speech_recognition.Speech_Recognition,None,name="speech recognition")
        return asr_task

    # 创建抓取零食的任务
    def create_task_grasp_snack(self,snack_rec_task:task.Task_image_rec,left_arm_container_rec_task:task.Task_image_rec, right_arm_container_rec_task:task.Task_image_rec,\
        task_left_arm_grap_container_dodge,task_right_arm_grap_container_dodge):
        # 抓取零食任务序列
        task_grasp_snack_seq = task.Task_sequence()
        
        # 抓取准备动作
        task_grasp_snack_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_snack_pre,None,utilis.Device_id.TBD,\
            [system.anchor_point.left_arm_snack_grip_pre,system.anchor_point.right_arm_snack_grip_pre],\
            target_clamps_status = arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ)
        task_grasp_snack_pre.parallel = task.Task.Task_parallel.ALL
        task_grasp_snack_pre.status = task.Task.Task_status.NOTREADY       # 需要参数
        task_grasp_snack_seq.add(task_grasp_snack_pre)
        
        # 抓取零食
        # 抓取坐标待定
        # TODO: 如果是一个手臂抓取两次零食, 则放置零食到抓取零食需要修改抓取方式
        task_grasp_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_snack,None,utilis.Device_id.TBD,\
            [system.anchor_point.left_arm_snack_grip,system.anchor_point.right_arm_snack_grip],\
            target_clamps_status  = arm.GripMethod.OPEN_CLOSE, arm_move_method = arm.ArmMoveMethod.YZ_X)
        task_grasp_snack.parallel = task.Task.Task_parallel.ALL
        task_grasp_snack.status   = task.Task.Task_status.NOTREADY       # 需要参数
        task_grasp_snack.add_predecessor_task(task_grasp_snack_pre)      # 添加前置任务
        task_grasp_snack_seq.add(task_grasp_snack)
        
        # 放置零食中间位置
        task_placement_snack_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_snack_pre,None,utilis.Device_id.TBD,\
            [system.anchor_point.left_arm_snack_placement_pre,system.anchor_point.right_arm_snack_placement_pre],\
            target_clamps_status = [arm.GripMethod.DONTCANGE, arm.GripMethod.DONTCANGE], arm_move_method = arm.ArmMoveMethod.X_Z_Y,\
            name="task_placement_snack_middle")
        task_placement_snack_pre.status   = task.Task.Task_status.NOTREADY  # 需要选择是左臂还是右臂
        task_placement_snack_pre.parallel = task.Task.Task_parallel.ALL
        task_placement_snack_pre.add_predecessor_task(task_grasp_snack)    # 添加前置任务
        task_grasp_snack_seq.add(task_placement_snack_pre)
        
        # 放置零食
        task_placement_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_snack,None,utilis.Device_id.TBD,\
            [system.anchor_point.left_arm_snack_placement,system.anchor_point.right_arm_snack_placement],\
            target_clamps_status = arm.GripMethod.OPEN, arm_move_method = arm.ArmMoveMethod.YZ_XANGLE)
        
        task_placement_snack.parallel = task.Task.Task_parallel.ALL
        task_placement_snack.status = task.Task.Task_status.NOTREADY              # 需要参数
        task_placement_snack.add_predecessor_task(task_placement_snack_pre)    # 添加前置任务
        task_grasp_snack_seq.add(task_placement_snack)
        
        # 夹容器前把夹零食任务做完
        task_left_arm_grap_container_dodge.add_predecessor_task(task_placement_snack)
        task_right_arm_grap_container_dodge.add_predecessor_task(task_placement_snack)
        
        # 将抓取零食任务与图像识别任务绑定
        snack_rec_task.add_need_modify_task(task_grasp_snack_pre)           # 抓零食准备位点
        snack_rec_task.add_need_modify_task(task_grasp_snack)               # 抓零食
        snack_rec_task.add_need_modify_task(task_placement_snack_pre)       # 放零食中间点
        snack_rec_task.add_need_modify_task(task_placement_snack)           # 放零食
        snack_rec_task.add_need_modify_task(task_left_arm_grap_container_dodge)   # 容器夹取准备动作
        snack_rec_task.add_need_modify_task(task_right_arm_grap_container_dodge)   # 容器夹取准备动作
        left_arm_container_rec_task.add_need_modify_task(task_placement_snack)
        right_arm_container_rec_task.add_need_modify_task(task_placement_snack)
        
        return task_grasp_snack_seq
    
    # !!!
    # 创建在零食桌前的任务
    def test_tasks_at_snack_desk(self,snack_list:order.Snack_list)->task.Task_sequence:
        tasks_pick_snack        = task.Task_sequence()
        
        #  将左臂抬到指定位置(食物框识别位置)
        task_left_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.LEFT, \
            system.anchor_point.left_arm_container_rec, arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="left arm move to rec container")
        task_left_arm_to_rec_contianer.parallel = task.Task.Task_parallel.ALL             # 可并行
        # task_left_arm_to_rec_contianer.add_predecessor_task(task_navigation_move_foward_to_snack_desk)    # 前置任务, 机器人移动到位
        tasks_pick_snack.add(task_left_arm_to_rec_contianer)
        
        #  将右臂抬到指定位置(食物框识别位置)
        task_right_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_container_rec, arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm move to rec container")
        task_right_arm_to_rec_contianer.parallel = task.Task.Task_parallel.ALL           # 可并行
        # task_right_arm_to_rec_contianer.add_predecessor_task(task_navigation_move_foward_to_snack_desk)  # 前置任务, 机器人移动到位
        tasks_pick_snack.add(task_right_arm_to_rec_contianer)
        
        #  左摄像头食物框识别(可前后并行，固定)
        task_left_camera_rec_container = task.Task_image_rec(task.Task_type.Task_image_rec.CONTAINER, None, utilis.Device_id.LEFT,\
            name="left camera rec container")
        task_left_camera_rec_container.parallel = task.Task.Task_parallel.ALL               # 可并行
        task_left_camera_rec_container.set_sleep_time_before_task(system.anchor_point.time_before_get_image) # 获取照片前暂停一下，以免图片模糊
        task_left_camera_rec_container.add_predecessor_task(task_left_arm_to_rec_contianer) # 前置任务, 左臂移动到食物框识别位置
        # task_left_camera_rec_container.add_predecessor_task(task_navigation_to_snack_desk)  # 前置任务, 导航到零食桌
        tasks_pick_snack.add(task_left_camera_rec_container)
        
        #  右摄像头食物框识别(可前后并行，固定)
        task_right_camera_rec_container = task.Task_image_rec(task.Task_type.Task_image_rec.CONTAINER, None, utilis.Device_id.RIGHT,\
            name="right camera rec container")
        task_right_camera_rec_container.parallel = task.Task.Task_parallel.ALL                  # 可并行
        task_right_camera_rec_container.set_sleep_time_before_task(system.anchor_point.time_before_get_image) # 获取照片前暂停一下，以免图片模糊
        task_right_camera_rec_container.add_predecessor_task(task_right_arm_to_rec_contianer)   # 前置任务, 右臂移动到食物框识别位置
        # task_right_camera_rec_container.add_predecessor_task(task_navigation_to_snack_desk)     # 前置任务, 导航到零食桌
        
        tasks_pick_snack.add(task_right_camera_rec_container)
        
        #  将左臂抬到零食识别位置(可前后并行，固定)
        task_left_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.LEFT, \
            system.anchor_point.left_arm_snack_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="left arm move to rec snack")
        task_left_arm_to_rec_snack.parallel = task.Task.Task_parallel.ALL                     # 可并行
        task_left_arm_to_rec_snack.add_predecessor_task(task_left_camera_rec_container)       # 前置任务, 左摄像头食物框识别
        tasks_pick_snack.add(task_left_arm_to_rec_snack)
        
        #  将右臂抬到零食识别位置(可前后并行，固定)
        task_right_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.RIGHT, \
            system.anchor_point.right_arm_snack_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ,\
                name="right arm move to rec snack")
        task_right_arm_to_rec_snack.parallel = task.Task.Task_parallel.ALL                    # 可并行
        task_right_arm_to_rec_snack.add_predecessor_task(task_right_camera_rec_container)     # 前置任务, 右摄像头食物框识别
        tasks_pick_snack.add(task_right_arm_to_rec_snack)
        
        #  左、右摄像头零食识别(不可并行，动态)
        task_rec_snack = task.Task_image_rec(task.Task_type.Task_image_rec.SNACK, None, utilis.Device_id.LEFT_RIGHT,\
            name="two arms rec snack")
        task_rec_snack.set_snack_list(snack_list)
        task_rec_snack.set_sleep_time_before_task(system.anchor_point.time_before_get_image)   # 获取照片前暂停一下，以免图片模糊
        task_rec_snack.add_predecessor_task(task_left_arm_to_rec_snack)                        # 前置任务, 左臂移动到零食识别位置
        task_rec_snack.add_predecessor_task(task_right_arm_to_rec_snack)                       # 前置任务, 右臂移动到零食识别位置
        tasks_pick_snack.add(task_rec_snack)
        
        # 左臂夹取零食框前的躲避动作
        task_left_arm_grap_container_dodge = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_dodge,None,utilis.Device_id.LEFT,\
                [copy.deepcopy(system.anchor_point.left_arm_container_grip_dodge)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="left arm grap container dodge")
        task_left_arm_grap_container_dodge.parallel = task.Task.Task_parallel.ALL          # 可并行
        task_left_arm_grap_container_dodge.status   = task.Task.Task_status.BEREADY        
        tasks_pick_snack.add(task_left_arm_grap_container_dodge)
        
        task_left_camera_rec_container.add_need_modify_task(task_left_arm_grap_container_dodge)  # 识别容器修改
        
        # 右臂夹取零食框前的躲避动作
        task_right_arm_grap_container_dodge = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_dodge,None,utilis.Device_id.RIGHT,\
                [copy.deepcopy(system.anchor_point.right_arm_container_grip_dodge)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="right arm grap container dodge")
        task_right_arm_grap_container_dodge.parallel = task.Task.Task_parallel.ALL          # 可并行
        task_right_arm_grap_container_dodge.status   = task.Task.Task_status.BEREADY        
        tasks_pick_snack.add(task_right_arm_grap_container_dodge)
        
        task_right_camera_rec_container.add_need_modify_task(task_right_arm_grap_container_dodge)  # 识别容器修改
        

        # 零食抓取任务
        snack_count = snack_list.get_all_snack_count()
        for i in range(snack_count):
            tasks_pick_snack.add(self.create_task_grasp_snack(task_rec_snack,task_left_camera_rec_container,task_right_camera_rec_container,\
                task_left_arm_grap_container_dodge,task_right_arm_grap_container_dodge))
        
        # 左臂夹取零食框, 准备动作
        task_left_arm_grap_container_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_pre,None,utilis.Device_id.LEFT,\
                [copy.deepcopy(system.anchor_point.left_arm_container_grip_pre)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="left arm grap container prepare")
        task_left_arm_grap_container_pre.parallel = task.Task.Task_parallel.ALL          # 可并行
        task_left_arm_grap_container_pre.status   = task.Task.Task_status.NOTREADY       # 需要参数
        task_left_arm_grap_container_pre.add_predecessor_task(task_left_arm_grap_container_dodge)   # 前置任务, 躲避动作
        task_left_arm_grap_container_pre.add_predecessor_task(task_right_arm_grap_container_dodge)  # 前置任务, 躲避动作
        tasks_pick_snack.add(task_left_arm_grap_container_pre)
        task_left_camera_rec_container.add_need_modify_task(task_left_arm_grap_container_pre)  # 识别容器绑定 夹取容器准备动作
        
        # 右臂夹取零食框, 准备动作
        task_right_arm_grap_container_pre    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container_pre,None,utilis.Device_id.RIGHT,\
                [copy.deepcopy(system.anchor_point.right_arm_container_grip_pre)],\
                    [arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.Z_XY,\
                        name="right arm grap container prepare")
        task_right_arm_grap_container_pre.parallel = task.Task.Task_parallel.ALL          # 可并行  
        task_right_arm_grap_container_pre.status   = task.Task.Task_status.NOTREADY       # 需要参数 
        task_right_arm_grap_container_pre.add_predecessor_task(task_left_arm_grap_container_dodge)   # 前置任务, 躲避动作
        task_right_arm_grap_container_pre.add_predecessor_task(task_right_arm_grap_container_dodge)  # 前置任务, 躲避动作
        tasks_pick_snack.add(task_right_arm_grap_container_pre)
        task_right_camera_rec_container.add_need_modify_task(task_right_arm_grap_container_pre) # 识别容器绑定 夹取容器
        
        
        # 功能暂停任务(等待全部零食抓取完毕)
        task_function_pause = task.Task_function(task.Task_type.Task_function.PAUSE, None,name="function pause")
        task_function_pause.add_predecessor_task(task_left_arm_grap_container_pre)  
        task_function_pause.add_predecessor_task(task_right_arm_grap_container_pre)  
        tasks_pick_snack.add(task_function_pause)
        
        # 左臂夹取零食框
        task_left_arm_grap_container    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container,None,utilis.Device_id.LEFT,\
                [copy.deepcopy(system.anchor_point.left_arm_container_grip)],\
                    [arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                        name="left arm grap container")
        task_left_arm_grap_container.parallel = task.Task.Task_parallel.ALL                  # 可并行
        task_left_arm_grap_container.add_predecessor_task(task_left_arm_grap_container_pre)  # 前置任务, 完成准备动作
        tasks_pick_snack.add(task_left_arm_grap_container)
            
        
        # 右臂夹取零食框
        task_right_arm_grap_container    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container,None,utilis.Device_id.RIGHT,\
                [copy.deepcopy(system.anchor_point.right_arm_container_grip)],\
                    [arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                        name="right arm grap container")
        task_right_arm_grap_container.parallel = task.Task.Task_parallel.ALL                   # 可并行  
        task_right_arm_grap_container.add_predecessor_task(task_right_arm_grap_container_pre)  # 前置任务, 完成准备动作
        tasks_pick_snack.add(task_right_arm_grap_container)
        
        
        #  左、右臂将零食框放到指定高度
        task_arm_dilivery_container   = task.Task_manipulation(task.Task_type.Task_manipulation.Deliever_container,None,utilis.Device_id.LEFT_RIGHT,\
                [system.anchor_point.left_arm_container_delivery,system.anchor_point.right_arm_container_delivery],\
                    [arm.GripMethod.DONTCANGE,arm.GripMethod.DONTCANGE], arm_move_method = arm.ArmMoveMethod.MODIFY_Z,\
                        name="two arms dilivery container")
        
        task_arm_dilivery_container.parallel = task.Task.Task_parallel.ALL
        task_arm_dilivery_container.set_subtask_count(2)  # 两个子任务
        task_arm_dilivery_container.add_predecessor_task(task_left_arm_grap_container)        # 前置任务, 左臂抓取容器
        task_arm_dilivery_container.add_predecessor_task(task_right_arm_grap_container)       # 前置任务, 右臂抓取容器
        tasks_pick_snack.add(task_arm_dilivery_container)
        
        # 机器人后退
        task_move_back_from_snack_desk = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,\
            system.anchor_point.snack_deck_move_back_pose, name="move back from snack desk")
        task_move_back_from_snack_desk.set_move_back_second(system.anchor_point.snack_deck_move_back_pose.run_time)  # 设置运行时间
        task_move_back_from_snack_desk.add_predecessor_task(task_right_arm_grap_container)   # 前置任务, 左臂抓取容器
        task_move_back_from_snack_desk.add_predecessor_task(task_left_arm_grap_container)    # 前置任务, 右臂抓取容器
        task_move_back_from_snack_desk.parallel = task.Task.Task_parallel.ALL
        tasks_pick_snack.add(task_move_back_from_snack_desk)

        
        return tasks_pick_snack
    
    # 创建放置容器的任务
    def test_tasks_lossen_container(self,table_id : utilis.Device_id):
        pass
    
    # 创建在饮料桌前的任务
    def test_tasks_at_drink_desk(self):
        pass
    
    # 创建放置饮料的任务
    def test_tasks_lossen_cup(self,table_id):
        pass


def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('assign_tasks')
    global system
    system = System()
    
    # 等待手臂,摄像头节点完成初始化
    
    left_arm_client = rospy.ServiceProxy(utilis.Topic_name.left_arm_prepare_service,std_srvs.Empty)
    right_arm_client = rospy.ServiceProxy(utilis.Topic_name.right_arm_prepare_service,std_srvs.Empty)
    camera_prepare_service = rospy.ServiceProxy(utilis.Topic_name.camera_prepare_service,std_srvs.Empty)
    asr_prepare_service = rospy.ServiceProxy(utilis.Topic_name.asr_prepare_service,std_srvs.Empty)

    if WAIT_FOR_ACTION_SERVER:
        rospy.loginfo("waiting for arm nodes...")
        left_arm_client.wait_for_service()
        right_arm_client.wait_for_service()
        rospy.loginfo("waiting for camera nodes...")
        camera_prepare_service.wait_for_service()
        rospy.loginfo("waiting for asr nodes...")
        asr_prepare_service.wait_for_service()
    
    # 自定义订单
    test_order_snack()

    # 新增识别服务
    # system.order_driven_task_schedul.add_asr_task()
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        rospy.loginfo("assign_tasks")
        # 按照设定的频率延时
        rate.sleep()
    # rospy.spin()
    
def ensure_directory_exists(path):
    if not os.path.exists(path):
        os.makedirs(path)

def test_order_snack():
    order_info = order.Order()
    order_info2 = order.Order()

    snack  = order.Snack(order.Snack.Snack_id.YIDA,1)
    order_info.add_snack(snack)
    
    snack  = order.Snack(order.Snack.Snack_id.CHENPIDAN,1)
    # order_info.add_snack(snack)
    
    snack  = order.Snack(order.Snack.Snack_id.GUODONG ,1)
    order_info.add_snack(snack)
    
    snack  = order.Snack(order.Snack.Snack_id.RUSUANJUN ,1)
    # order_info.add_snack(snack)
    
    drink  = order.Drink(order.Drink.Drink_id.COFFEE,1)

    # order_info.add_snack(snack2)
    order_info.order_id = 2
    order_info.table_id = utilis.Device_id.RIGHT
    # order_info.table_id = utilis.Device_id.LEFT
    
    order_info2.add_drink(drink)
    order_info2.order_id = 3
    # order_info2.table_id = utilis.Device_id.RIGHT
    order_info2.table_id = utilis.Device_id.LEFT

    # tasks = system.order_driven_task_schedul.add_task(order_info)
    # tasks2 = system.order_driven_task_schedul.add_task(order_info2)
    # tasks2 = system.order_driven_task_schedul.add_task(order_info2)

    tasks_get_snack = system.order_driven_task_schedul.test_tasks_at_snack_desk(order_info.snack_list)
    tasks_get_snack.update_group_id(6)
    
    # tasks_lossen_snack = system.order_driven_task_schedul.test_tasks_lossen_container(order_info.table_id)
    # tasks_lossen_snack.update_group_id(7)
    
    # tasks_get_drink = system.order_driven_task_schedul.test_tasks_at_drink_desk()
    # tasks_get_drink.update_group_id(8)
    
    # task_lossen_cup = system.order_driven_task_schedul.test_tasks_lossen_cup(order_info.table_id)
    # task_lossen_cup.update_group_id(9)

    # log.log_tasks_info(tasks_get_snack,"new_all_task.log")
    
    system.order_driven_task_schedul.task_manager.waiting_task.add(tasks_get_snack)
    # system.order_driven_task_schedul.task_manager.waiting_task.add(tasks_lossen_snack)
    # system.order_driven_task_schedul.task_manager.waiting_task.add(tasks_get_drink)
    # system.order_driven_task_schedul.task_manager.waiting_task.add(task_lossen_cup)
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
