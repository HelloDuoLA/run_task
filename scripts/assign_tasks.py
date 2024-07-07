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


# 初始化
class System():
    instance = None          # 句柄
    # 初始化
    def __init__(self):
        System.instance = self
        self.anchor_point = self.Anchor_point()
        
        rospy.loginfo(f"node: {rospy.get_name()}, system init")
        # 执行器初始化
        rospy.loginfo(f"node: {rospy.get_name()}, navigation_actuator")
        self.navigation_actuator  =  Navigation_actuator()  # 导航执行器
        rospy.loginfo(f"node: {rospy.get_name()}, image_rec_actuator")
        self.image_rec_actuator   =  Image_rec_actuator()   # 图像识别执行器
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator_actuator")
        self.manipulator_actuator =  Manipulator_actuator() # 机械臂执行器
        
        # 机器人状态
        self.robot                =  robot.Robot()           # 机器人
        
        # 任务管理器
        self.task_manager = Task_manager(self.robot)
        
        # 订单驱动任务增加器
        self.order_driven_task_schedul     = Order_driven_task_schedul(self.task_manager)
        
        # 图像识别驱动任务修改器
        self.image_rec_driven_task_schedul = Image_rec_driven_task_schedul(self.task_manager)
        
        # 设置初始位姿
        # TODO:调试需要,暂时注释
        # self.set_initial_pose()
    
    # 设置初始位姿
    def set_initial_pose(self):
        # 获得初始位姿
        x = rospy.get_param('~InitialPose/position_x')
        y = rospy.get_param('~InitialPose/position_y')
        yaw = rospy.get_param('~InitialPose/yaw')
        rospy.loginfo(f"node: {rospy.get_name()}, get params x:{x} y:{y} yaw: {yaw}")
        
        # 设置初始位置
        initial_pose = utilis.Pose2D(x,y,yaw)
        self.robot.update_robot_pose3d(utilis.Pose3D.instantiate_by_pose2d(initial_pose))
        self.set_amcl_pose(initial_pose)
    
    # 初始化AMCL位姿
    def set_amcl_pose(self,pose:utilis.Pose2D):
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
    
        rospy.loginfo(f"node: {rospy.get_name()}, orientation: {q}")
        # 发布初始位置
        pub.publish(initial_pose)
        
    # 常量定义
    class Anchor_point():
        def __init__(self):
            self._initialize_robot_anchor_point()
            self._initialize_arm_anchor_point()
        # 初始化机器人位点常量配置
        def _initialize_robot_anchor_point(self):
            # 初始点
            self.map_initial_pose =  _get_robot_anchor_pose("InitialPose")
            # 零食桌
            self.map_snack_desk   =  _get_robot_anchor_pose("SnackDesk")
            # 饮料桌
            self.map_drink_desl   =  _get_robot_anchor_pose("DrinkDesk")
            # 右服务台位姿
            self.map_right_service_desk =  _get_robot_anchor_pose("RightServiceDesk")
            # 左服务台位姿
            self.map_left_service_desk  =  _get_robot_anchor_pose("LeftServiceDesk")
            
            # 通过名字获取机器人定位点
            def _get_robot_anchor_pose(anchor_point_name):
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
            self.left_arm_idle                 = _get_arm_anchor_angle("LeftArmIdle")                # 左臂闲置
            self.left_arm_container_rec        = _get_arm_anchor_angle("LeftArmContainerRec")        # 左臂识别容器
            self.left_arm_snack_rec            = _get_arm_anchor_angle("LeftArmSnackRec")            # 左臂识别零食
            self.left_arm_snack_placement      = _get_arm_anchor_coord("LeftArmSnackPlacement")     # 左臂零食放置
            self.left_arm_container_delivery   = _get_arm_anchor_coord("LeftArmContainerDelivery")  # 左臂容器运送时的姿态
            self.left_arm_container_placement  = _get_arm_anchor_coord("LeftArmContainerPlacement") # 左臂容器放置
            self.left_arm_coffee_machine_rec   = _get_arm_anchor_angle("LeftArmCoffeeMachineRec")    # 左臂咖啡机识别
            self.left_arm_machine_turn_on_pre  = _get_arm_anchor_coord("LeftArmMachineTurnOnPre")   # 左臂咖啡机预备动作
            self.left_arm_machine_turn_off_pre = _get_arm_anchor_coord("LeftArmMachineTurnOffPre")  # 左臂咖啡机预备动作
            
            
            self.right_arm_idle                = _get_arm_anchor_angle("RightArmIdle")               # 右臂空闲
            self.right_arm_container_rec       = _get_arm_anchor_angle("RightArmContainerRec")       # 右臂识别容器
            self.right_arm_snack_rec           = _get_arm_anchor_angle("RightArmSnackRec")           # 右臂识别零食
            self.right_arm_snack_placement     = _get_arm_anchor_coord("RightArmSnackPlacement")    # 左臂零食放置
            self.right_arm_container_delivery  = _get_arm_anchor_coord("RightArmContainerDelivery") # 右臂容器运送时的姿态
            self.right_arm_container_placement = _get_arm_anchor_coord("RightArmContainerPlacement")# 右臂容器放置
            self.right_arm_cup_rec_pre         = _get_arm_anchor_angle("RightArmCupRecPre")          # 右臂杯子识别预备
            self.right_arm_cup_rec             = _get_arm_anchor_angle("RightArmCupRec")             # 右臂杯子识别
            self.right_arm_cup_grab            = _get_arm_anchor_coord("RightArmCupGrab")           # 右臂杯子夹取
            self.right_arm_cup_water           = _get_arm_anchor_coord("RightArmCupWater")          # 右臂杯子接水
            self.right_arm_cup_delivery        = _get_arm_anchor_coord("CupDelivery")               # 右臂杯子运送
            self.right_arm_cup_placement       = _get_arm_anchor_coord("CupPlacement")              # 右臂杯子放置

            # 通过名字获取机械臂定位点
            def _get_arm_anchor_coord(anchor_point_name):
                if "left" in  anchor_point_name.lower() :
                    arm_id = utilis.Device_id.LEFT
                elif "right" in  anchor_point_name.lower() :
                    arm_id = utilis.Device_id.RIGHT
                else:
                    arm_id = utilis.Device_id.TBD
                pose = arm.Arm_pose(rospy.get_param(f'~{anchor_point_name}/coords'),arm.PoseType.BASE_COORDS,arm_id)
                return pose
            
            # 通过名字获取机械臂定位点(角度)
            def _get_arm_anchor_angle(anchor_point_name):
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
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # TODO:调试需要,暂时注释
        self.ac.wait_for_server()
    # 运行
    def run(self, navigation_task:task.Task_navigation):
        self.task = navigation_task
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = self.task.target_3D_pose.x
        goal.target_pose.pose.position.y = self.task.target_3D_pose.y
        goal.target_pose.pose.position.z = 0
        orientation = quaternion_from_euler(0, 0, self.task.target_3D_pose.yaw)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        self.task.update_start_status() # 刷新开始时间
        self.ac.send_goal(goal,self.navigation_task_done_callback,self.navigation_task_active_callback,self.navigation_task_feedback_callback)
    
    # 完成回调
    @staticmethod
    def navigation_task_done_callback(status, result):
        # PENDING    = 0  # 目标已被接受，但处理尚未开始
        # ACTIVE     = 1  # 目标正在被处理中
        # PREEMPTED  = 2  # 目标在达成之前被另一个目标取代，或者在目标完成之前被取消。
        # SUCCEEDED  = 3  # 目标已成功完成
        # ABORTED    = 4  # 目标在完成前被中止，但不是因为外部的取消请求。
        # REJECTED   = 5  # 目标被拒绝，不会被执行。
        # PREEMPTING = 6  # 目标正在被取代之前的过程中。
        # RECALLING  = 7  # 目标正在被取消之前的过程中，但尚未开始执行。
        # RECALLED   = 8  # 目标已被成功取消，在开始执行之前。
        # LOST       = 9  # 目标被认为丢失。
        rospy.loginfo(f"node: {rospy.get_name()}, navigation done. status:{status} result:{result}")
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"node: {rospy.get_name()}, navigation succeed. status : {status}")
            system.navigation_actuator.task.update_end_status(task.Task.Task_result.SUCCEED)
        else:
            rospy.loginfo(f"node: {rospy.get_name()}, navigation failed. status : {status}")
            system.navigation_actuator.task.update_end_status(task.Task.Task_result.FAILED)
        
        # 任务自带的回调
        if system.navigation_actuator.task.finish_cb is not None:
            system.navigation_actuator.task.finish_cb(status, result)
        
        # 给任务管理器的回调
        system.task_manager.tm_task_finish_callback(system.navigation_actuator.task, status, result)
    
    # 激活回调
    @staticmethod
    def navigation_task_active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, navigation active")

    # 反馈回调
    @staticmethod
    def navigation_task_feedback_callback(feedback:MoveBaseFeedback):
        global system
        pose = feedback.base_position.pose
        pose3D = utilis.Pose3D.instantiate_by_geometry_msg(pose)
        rospy.loginfo(f"node: {rospy.get_name()}, navigation feedback. pose:x = {pose3D.x} y = {pose3D.y} yaw = {pose3D.yaw}")
        # !调试阶段无法进行位姿更新
        system.robot.update_robot_pose3d(pose3D) # 更新机器人位姿
    
# 机械臂执行器 
class Manipulator_actuator():
    def __init__(self):
        self.left_arm_ac  = actionlib.SimpleActionClient(utilis.Topic_name.left_arm_action,  msg.MoveArmAction)
        self.right_arm_ac = actionlib.SimpleActionClient(utilis.Topic_name.right_arm_action, msg.MoveArmAction)
        # TODO:调试需要,暂时注释
        # self.left_arm_ac.wait_for_server()
        # self.right_arm_ac.wait_for_server()
        self.running_tasks_manager = task.Task_manager_in_running() # 正在执行的任务管理器
    
    # 运行
    def run(self, manipulation_task:task.Task_manipulation):
        # 加在运行序列中
        task_index = self.running_tasks_manager.add_task(manipulation_task)
        
        # 任务开始
        manipulation_task.start_time()

        # 单臂
        if manipulation_task.arm_id == utilis.Device_id.LEFT:
            # 设置机械臂状态
            system.robot.update_arm_status(manipulation_task.arm_id,robot.manipulation_status.arm.status.BUSY)
            # 设置action 目标
            goal                      = msg.MoveArmGoal()
            goal.task_index           = task_index
            goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0]
            goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            goal.grasp_first          = manipulation_task.clamp_first
            goal.grasp_speed          = manipulation_task.clamp_speed
            self.left_arm_ac.send_goal(goal,self.done_callback,self.active_callback,self.feedback_callback)
        
        elif manipulation_task.arm_id == utilis.Device_id.RIGHT:
            # 设置机械臂状态
            system.robot.update_arm_status(manipulation_task.arm_id,robot.manipulation_status.arm.status.BUSY)
            # 设置action 目标
            goal                      = msg.MoveArmGoal()
            goal.task_index           = task_index
            goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0]
            goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            goal.grasp_first          = manipulation_task.clamp_first
            goal.grasp_speed          = manipulation_task.clamp_speed
            self.right_arm_ac.send_goal(goal,self.done_callback,self.active_callback,self.feedback_callback)
            
        elif manipulation_task.arm_id == utilis.Device_id.LEFT_RIGHT:
            left_goal              = msg.MoveArmGoal()
            right_goal             = msg.MoveArmGoal()
            for i in range(2):
                if manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.LEFT:
                    system.robot.update_arm_status(utilis.Device_id.LEFT,robot.manipulation_status.arm.status.BUSY)
                    left_goal.task_index           = task_index
                    left_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i]
                    left_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    left_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    left_goal.grasp_first          = manipulation_task.clamp_first
                    left_goal.grasp_speed          = manipulation_task.clamp_speed
                    
                elif manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.RIGHT:
                    system.robot.update_arm_status(utilis.Device_id.RIGHT,robot.manipulation_status.arm.status.BUSY)
                    right_goal.task_index           = task_index
                    right_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i]
                    right_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    right_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    right_goal.grasp_first          = manipulation_task.clamp_first
                    right_goal.grasp_speed          = manipulation_task.clamp_speed
            
            self.left_arm_ac.send_goal(left_goal,self.done_callback,self.active_callback,self.feedback_callback)
            self.right_arm_ac.send_goal(right_goal,self.done_callback,self.active_callback,self.feedback_callback)


    # 完成回调
    @staticmethod
    def done_callback(status, result:msg.MoveArmResult):
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator done. status:{status} result:{result}")
        task_current =  system.manipulator_actuator.running_tasks_manager.get_task(result.task_index)
        # 任务成功
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"node: {rospy.get_name()}, manipulator succeed. status : {status}")
            task_current.update_end_status(task.Task.Task_result.SUCCEED)
        # 任务失败
        # TODO: 需要分析是机械臂移动失败还是夹取失败
        else:
            rospy.loginfo(f"node: {rospy.get_name()}, manipulator failed. status : {status}")
            task_current.update_end_status(task.Task.Task_result.FAILED)
        
        # 任务自带的回调
        task_current.finish_cb(status, result)
        
        # 给任务管理器的回调
        system.task_manager.tm_task_finish_callback(system.manipulator_actuator.task, status, result)
    
    # 激活回调
    @staticmethod
    def active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator active")
    
    # 反馈回调
    @staticmethod
    def feedback_callback(feedback:msg.MoveArmFeedback):
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator feedback. {feedback}")


# 图像识别任务执行器
# TODO:修改为可并行
class Image_rec_actuator():
    # instance = None
    def __init__(self):
        # image_rec_actuator.instance  = self 
        self.running_tasks_manager          = task.Task_manager_in_running()  # 正在执行的任务管理器       
        self.pub = rospy.Publisher (utilis.Topic_name.image_recognition_request,msg.ImageRecRequest,self,queue_size=10) # 发布识别任务
        self.sub = rospy.Subscriber(utilis.Topic_name.image_recognition_result,msg.ImageRecResult,queue_size=10)        # 订阅识别结果

    # 运行
    def run(self, task_image_rec_task:task.Task_image_rec):
        task_index = self.running_tasks_manager.add_task(task_image_rec_task)
        # 发布任务
        task_info = msg.ImageRecRequest()
        task_info.task_index = task_index                    # 任务索引
        # 如果是识别零食, 则需要给出零食列表
        if task_index == task.Task_type.Task_image_rec.SNACK:
            task_info.snacks = task_image_rec_task.snack_list.to_msg() # 零食列表
        task_info.task_type  = task_image_rec_task.task_type # 任务类型
        task_info.arm_poses  = system.robot.get_msg_arms_pose_with_id(task_image_rec_task.camera_id) # 机械臂位姿
        
        # 发布消息
        self.pub.publish(task_info)
        
        # TODO: 修改任务状态为等待消息回传
    
    # 识别结果话题回调
    @staticmethod
    def do_image_rec_result_callback(result:msg.ImageRecResult):
        # 获取对应的服务对象
        task_obj = system.image_rec_actuator.running_tasks_manager.get_task(result.task_index)
        # 根据不同任务作出不同处理
        # TODO:待处理
        # 识别零食
        if task_obj.task_type == task.Task_type.Task_image_rec.SNACK:
            pass
        # 识别容器
        elif task_obj.task_type == task.Task_type.Task_image_rec.CONTIANER:
            pass
        # 识别咖啡机
        elif task_obj.task_type == task.Task_type.Task_image_rec.COFFEE_MACHIE_SWITCH:
            pass
        # 识别杯子
        elif task_obj.task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE:
            pass
        else:
            raise ValueError("Invalid task type")
        # 任务自带的回调
        task_obj.finish_cb(actionlib.GoalStatus.SUCCEEDED)
        # 给任务管理器的回调
        system.task_manager.tm_task_finish_callback(task_obj, actionlib.GoalStatus.SUCCEEDED)
    

# 任务管理器
class Task_manager():
    def __init__(self,robot=None):
        self.robot            = robot                 # 执行任务的机器人
        self.finished_tasks   = task.Task_sequence()  # 已经完成的任务列表
        self.executed_tasks   = task.Task_sequence()  # 正在执行的任务列表
        # self.conflicting_task = task.Task_sequence()  # 冲突的任务列表(可以并行, 但因为硬件冲突暂时无法并行)
        self.waiting_task     = task.Task_sequence()  # 等待执行的任务列表
        
        # 每0.5s执行一次任务
        timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
    
    # 任务完成回调
    def tm_task_finish_callback(self, task, status, result):
        rospy.loginfo(f"node: {rospy.get_name()}, task_manager, tasks: {task}")
    

    # 定时器任务
    @staticmethod
    def timer_callback(event):
        rospy.loginfo("Task manager timer callback")
        for current_task in system.task_manager.waiting_task.task_list:
            if system.task_manager.task_is_ready_to_run(current_task):
                # 导航任务
                if current_task.task_type == task.Task_type.Task_navigate:
                    system.robot.update_robot_status(robot.Robot.Robot_status.MOVING)
                    system.navigation_actuator.run(task)
                # 机械臂任务
                elif current_task.task_type == task.Task_type.Task_manipulation:
                    # 设置状态
                    if current_task.arm_id == utilis.Device_id.LEFT_RIGHT:
                        system.robot.update_arm_status(utilis.Device_id.LEFT, task.Task.Task_status.RUNNING)
                        system.robot.update_arm_status(utilis.Device_id.RIGHT,task.Task.Task_status.RUNNING)
                    else:
                        system.robot.update_arm_status(current_task.arm_id,task.Task.Task_status.RUNNING)
                    system.manipulator_actuator.run(task)
                # 图像识别任务
                elif current_task.task_type == task.Task_type.Task_image_rec:
                    # 设置状态
                    if current_task.camera_id == utilis.Device_id.LEFT_RIGHT:
                        system.robot.update_arm_status(utilis.Device_id.LEFT, task.Task.Task_status.RUNNING)
                        system.robot.update_arm_status(utilis.Device_id.RIGHT,task.Task.Task_status.RUNNING)
                    else:
                        system.robot.update_arm_status(current_task.camera_id,task.Task.Task_status.RUNNING)
                    # 运行
                    system.image_rec_actuator.run(task)
    # 判断任务是否能够运行
    def task_is_ready_to_run(self,current_task:task.Task):
        if current_task.status == task.Task.Task_status.NOTREADY:
            return False
        else:
            # 正在执行的任务为0
            if self.executed_tasks.get_task_count == 0:
                return True
            # 正在执行的任务不为0
            else:
                # 任务不允许并行
                if current_task.parallel == task.Task.Task_parallel.NOTALLOWED:
                    return False
                elif current_task.parallel == task.Task.Task_parallel.ALL:
                    pass
                    # 判断前置任务是否完成
                    # 判断资源是否允许

        return True

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
    
    # 根据订单更新任务
    def update_task(self,order:order.Order):
        result = False
        if order.operation == order.Operation.ADD:
            rospy.loginfo(f"node name {rospy.get_name()}, order driven task schedul, add new order")
            result = self.add_task(order)
        elif order.operation == order.Operation.DELETE:
            result = self.delete_task(order)
        elif order.operation == order.Operation.MODIFY:
            pass
        elif order.operation == order.Operation.CHECK:
            pass
        else:
            rospy.logerr(f"node name {rospy.get_name()}, order driven task schedul, invalid operation")
        return result
    
    # 新增任务
    def add_task(self,order:order.Order): 
        new_task_sequence = task.Task_sequence()
        
        # 有零食请求
        if order.has_snack_request:
            new_task_sequence.add(self.create_tasks_before_grasp_snack(order.snack_list))
            snack_count = order.get_all_snack_count()
            task_image_rec_snack = new_task_sequence.task_list[-1]
            for i in range(snack_count):
                new_task_sequence.add(self.create_task_grasp_snack(task_image_rec_snack))
            new_task_sequence.add(self.create_tasks_after_grasp_snack(order.table_id))
        
        # 有饮料请求
        if order.has_drink_request:
            new_task_sequence.add(self.create_tasks_before_get_drink())
            # snack_count = order.get_all_snack_count()
            # new_task_sequence.add(self.task_grasp_snack_seq * snack_count)
            new_task_sequence.add(self.create_tasks_after_get_drink(order.table_id))
        
        # 更新组ID
        new_task_sequence.update_group_id(order.order_id)
        
        # 添加到任务管理器待执行队列 
        self.task_manager.waiting_task.add(new_task_sequence)
        # 新增任务输出到指定文件
        log.log_add_tasks_info(new_task_sequence)
        # rospy.loginfo(f"new_task_sequence {new_task_sequence}")
        # print(context)
        
    # 删除任务
    # TODO:需要考虑任务正在执行了怎么办
    def delete_task(self,task):
        pass
        
    #  拿零食前准备
    def create_tasks_before_grasp_snack(self,snack_list:order.Snack_list):
        tasks_before_pick_snack        = task.Task_sequence()
        
        #  手臂移到空闲位, 并关闭夹爪(不可并行，固定)
        task_left_right_arms_idle      = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE, None, utilis.Device_id.LEFT_RIGHT, [system.anchor_point.left_arm_idle,system.anchor_point.right_arm_idle],\
            [robot.manipulation_status.clamp.status.CLOSE,robot.manipulation_status.clamp.status.CLOSE])
        tasks_before_pick_snack.add(task_left_right_arms_idle)
        
        #  前往零食桌(不可并行，固定)
        task_navigation_to_snack_desk  = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_snack_desk, None, system.anchor_point.map_snack_desk)
        tasks_before_pick_snack.add(task_navigation_to_snack_desk)
        
        #  将左臂抬到指定位置(食物框识别位置)(可前后并行，固定)
        task_left_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.LEFT, system.anchor_point.left_arm_container_rec)
        task_left_arm_to_rec_contianer.parallel = task.Task.Task_parallel.ALL
        tasks_before_pick_snack.add(task_left_arm_to_rec_contianer)
        
        #  将右臂抬到指定位置(食物框识别位置)(可前后并行，固定)
        task_right_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.RIGHT, system.anchor_point.right_arm_container_rec)
        task_right_arm_to_rec_contianer.parallel = task.Task.Task_parallel.ALL
        tasks_before_pick_snack.add(task_right_arm_to_rec_contianer)
        
        #  左摄像头食物框识别(可前后并行，固定)
        task_left_camera_rec_container = task.Task_image_rec(task.Task_type.Task_image_rec.CONTAINER, None, utilis.Device_id.LEFT)
        task_left_camera_rec_container.parallel = task.Task.Task_parallel.ALL
        tasks_before_pick_snack.add(task_left_camera_rec_container)
        
        #  右摄像头食物框识别(可前后并行，固定)
        task_right_camera_rec_container = task.Task_image_rec(task.Task_type.Task_image_rec.CONTAINER, None, utilis.Device_id.RIGHT)
        task_right_camera_rec_container.parallel = task.Task.Task_parallel.ALL
        tasks_before_pick_snack.add(task_right_camera_rec_container)
        
        #  将左臂抬到零食识别位置(可前后并行，固定)
        task_left_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.LEFT, system.anchor_point.left_arm_snack_rec)
        task_left_arm_to_rec_snack.parallel = task.Task.Task_parallel.ALL
        tasks_before_pick_snack.add(task_left_arm_to_rec_snack)
        
        #  将右臂抬到零食识别位置(可前后并行，固定)
        task_right_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.RIGHT, system.anchor_point.right_arm_snack_rec)
        task_right_arm_to_rec_snack.parallel = task.Task.Task_parallel.ALL
        tasks_before_pick_snack.add(task_right_arm_to_rec_snack)
        
        #  左、右摄像头零食识别(不可并行，动态)
        task_rec_snack = task.Task_image_rec(task.Task_type.Task_image_rec.SNACK, None, utilis.Device_id.LEFT_RIGHT)
        task_rec_snack.set_snack_list(snack_list)
        tasks_before_pick_snack.add(task_rec_snack)
        
        return tasks_before_pick_snack
    
    #  拿零食后
    def create_tasks_after_grasp_snack(self,table_id:utilis.Device_id):
        #  把左,右臂放到指定位置(不可并行，固定),并夹取食物框
        tasks_after_pick_snack        = task.Task_sequence()
        task_grap_container           = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container,None,utilis.Device_id.LEFT_RIGHT,\
                [system.constant_config.arm_anchor_point.left_arm_container_grasp,system.constant_config.arm_anchor_point.right_arm_container_grasp],\
                    [robot.manipulation_status.clamp.status.CLOSE,robot.manipulation_status.clamp.status.CLOSE])
        tasks_after_pick_snack.add(task_grap_container)
        
        #  左、右臂将零食框放到指定高度(可后并行，固定)
        task_arm_dilivery_container   = task.Task_manipulation(task.Task_type.Task_manipulation.Deliever_container,None,utilis.Device_id.LEFT_RIGHT,\
                [system.constant_config.arm_anchor_point.left_arm_container_delivery,system.constant_config.arm_anchor_point.right_arm_container_delivery])
        task_arm_dilivery_container.parallel = task.Task.Task_parallel.ALL
        tasks_after_pick_snack.add(task_arm_dilivery_container)
        
        # 机器人原地转身(可前并行，固定)
        task_rotate = task.Task_navigation(task.Task_type.Task_navigate.Rotation_in_place,None,rotation_degree=20)
        task_rotate.parallel = task.Task.Task_parallel.ALL
        tasks_after_pick_snack.add(task_rotate)
        
        #  导航前往n号桌(不可并行，半动态)
        if table_id == utilis.Device_id.LEFT.value:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_left_service_desk,None,system.constant_config.robot_anchor_point.left_service_desk)
        elif table_id == utilis.Device_id.RIGHT.value:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_right_service_desk,None,system.constant_config.robot_anchor_point.right_service_desk)
        else:
            raise ValueError("Invalid table_id")
        task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_one_service_desk,None)
        task_navigation_to_service_desk.parallel = task.Task.Task_parallel.NOTALLOWED
        tasks_after_pick_snack.add(task_navigation_to_service_desk)
        
        #  将左、右臂放到指定位置后，松开(不可并行，固定)
        task_arm_placement_container   = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_container,None,utilis.Device_id.LEFT_RIGHT,\
                [system.constant_config.arm_anchor_point.left_arm_container_placement,system.constant_config.arm_anchor_point.right_arm_container_placement],\
                [robot.manipulation_status.clamp.status.OPEN,robot.manipulation_status.clamp.status.OPEN])
        tasks_after_pick_snack.add(task_arm_placement_container)
    
        #  将左,右臂放到空闲位置(可并行，固定)
        task_arm_idle   = task.Task_manipulation(task.Task_type.Task_manipulation.Move,None,utilis.Device_id.LEFT_RIGHT,\
                [system.constant_config.arm_anchor_point.right_arm_idle,system.constant_config.arm_anchor_point.right_arm_idle],\
                [robot.manipulation_status.clamp.status.OPEN,robot.manipulation_status.clamp.status.OPEN])
        task_arm_idle.parallel = task.Task.Task_parallel.ALL
        tasks_after_pick_snack.add(task_arm_idle)
        
        # 机器人原地转身(可前并行，固定)
        task_rotate2 = task.Task_navigation(task.Task_type.Task_navigate.Rotation_in_place,None,rotation_degree=20)
        task_rotate2.parallel = task.Task.Task_parallel.ALL
        tasks_after_pick_snack.add(task_rotate2)
        
        # 赋值
        return tasks_after_pick_snack
        
    # 拿饮料前
    def create_tasks_before_get_drink(self):
        tasks_before_get_drink        = task.Task_sequence()
        
        #  导航前往饮料桌(不可并行，固定)
        task_navigation_to_drink_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_drink_desk,None,system.constant_config.robot_anchor_point.drink_desk)
        tasks_before_get_drink.add(task_navigation_to_drink_desk)
        
        #  左臂抬到指定位置(可前后并行，固定)
        task_left_arm_to_rec_coffee_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, system.constant_config.arm_anchor_point.coffee_machine_rec)
        task_left_arm_to_rec_coffee_machine.parallel = task.Task.Task_parallel.ALL
        tasks_before_get_drink.add(task_left_arm_to_rec_coffee_machine)

        # #  右臂抬到指定位置(可前后并行，固定)
        # task_right_arm_to_rec_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT,system.constant_config.arm_anchor_point.cup_rec)
        # task_right_arm_to_rec_cup.parallel = task.Task.Task_parallel.ALL
        # tasks_before_get_drink.add(task_right_arm_to_rec_cup)
        
        #  左臂图像识别(咖啡机开关位置)(可并行，固定)
        task_left_camera_rec_coffee_machine = task.Task_image_rec(task.Task_type.Task_image_rec.COFFEE_MACHIE_SWITCH,None,utilis.Device_id.LEFT)
        task_left_camera_rec_coffee_machine.parallel = task.Task.Task_parallel.ALL    
        task_left_camera_rec_coffee_machine.add_predecessor_task(task_left_arm_to_rec_coffee_machine) # 绑定前置任务
        tasks_before_get_drink.add(task_left_camera_rec_coffee_machine)
        
        #  头摄像头图像识别(杯子位置、咖啡机位置)(可并行，固定)
        task_head_camera_rec_cup = task.Task_image_rec(task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE,None,utilis.Device_id.HEAD)
        task_head_camera_rec_cup.parallel = task.Task.Task_parallel.ALL
        tasks_before_get_drink.add(task_head_camera_rec_cup)
        
        #  右臂夹取杯子(可并行，固定)
        task_right_arm_grasp_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_cup,None,utilis.Device_id.RIGHT,\
            system.constant_config.arm_anchor_point.cup_grab,robot.manipulation_status.clamp.status.CLOSE)
        task_right_arm_grasp_cup.parallel = task.Task.Task_parallel.ALL
        task_right_arm_grasp_cup.status   = task.Task.Task_status.NOTREADY  # 需要参数
        tasks_before_get_drink.add(task_right_arm_grasp_cup)
        task_head_camera_rec_cup.add_need_modify_task(task_right_arm_grasp_cup) # 绑定头部识别任务
        
        # 左臂放置到按钮下方
        task_left_arm_prepare_turn_on_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Move,None,utilis.Device_id.LEFT)
        task_left_arm_prepare_turn_on_machine.parallel = task.Task.Task_parallel.ALL
        task_left_arm_prepare_turn_on_machine.status   = task.Task.Task_status.NOTREADY  # 需要参数
        tasks_before_get_drink.add(task_left_arm_prepare_turn_on_machine)
        task_left_camera_rec_coffee_machine.add_need_modify_task(task_left_arm_prepare_turn_on_machine) # 绑定左臂识别任务
        
        # 左臂向上触碰按钮
        task_left_arm_turn_on_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_on_coffee_machine,None,utilis.Device_id.LEFT)
        task_left_arm_turn_on_machine.parallel = task.Task.Task_parallel.ALL
        task_left_arm_turn_on_machine.status   = task.Task.Task_status.NOTREADY  # 需要参数
        task_left_arm_turn_on_machine.add_predecessor_task(task_left_arm_prepare_turn_on_machine) # 绑定前置任务
        tasks_before_get_drink.add(task_left_arm_turn_on_machine)
        task_left_camera_rec_coffee_machine.add_need_modify_task(task_left_arm_turn_on_machine) # 绑定左臂识别任务
        
        
        #  右臂将杯子挪到咖啡机(可并行,固定)
        task_right_arm_water_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Move,None,utilis.Device_id.RIGHT,\
            system.constant_config.arm_anchor_point.cup_water)
        task_right_arm_water_cup.parallel = task.Task.Task_parallel.ALL
        task_right_arm_water_cup.status   = task.Task.Task_status.NOTREADY
        tasks_before_get_drink.add(task_right_arm_water_cup)
        task_head_camera_rec_cup.add_need_modify_task(task_right_arm_water_cup) #绑定头部识别任务
        
        # 左臂放置到按钮上方
        task_left_arm_prepare_turn_off_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Move,None,utilis.Device_id.LEFT)
        task_left_arm_prepare_turn_off_machine.parallel = task.Task.Task_parallel.ALL
        task_left_arm_prepare_turn_off_machine.status   = task.Task.Task_status.NOTREADY  # 需要参数
        tasks_before_get_drink.add(task_left_arm_prepare_turn_off_machine)
        task_left_camera_rec_coffee_machine.add_need_modify_task(task_left_arm_prepare_turn_off_machine) # 绑定左臂识别任务
        
        #  左臂向下触碰按钮(不可并行，动态)
        task_left_arm_turn_off_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_off_coffee_machine,None,utilis.Device_id.LEFT)
        task_left_arm_turn_off_machine.parallel = task.Task.Task_parallel.ALL
        task_left_arm_turn_off_machine.status   = task.Task.Task_status.NOTREADY  # 需要参数
        task_left_arm_turn_off_machine.add_predecessor_task(task_left_arm_prepare_turn_on_machine) # 绑定前置任务
        tasks_before_get_drink.add(task_left_arm_turn_off_machine)
        task_left_camera_rec_coffee_machine.add_need_modify_task(task_left_arm_turn_off_machine) # 绑定左臂识别任务
        
        # TODO: 接咖啡的任务需要根据实际调整,  还需要等待一段时间.

        # 赋值
        return tasks_before_get_drink

    # 拿饮料后
    def create_tasks_after_get_drink(self,table_id:utilis.Device_id):
        tasks_after_get_drink        = task.Task_sequence()
        
        #  将左臂抬到指定位置(可并行，固定)
        task_left_arm_idle = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE,None,utilis.Device_id.LEFT,\
            system.anchor_point.left_arm_idle,robot.manipulation_status.clamp.status.CLOSE)
        task_left_arm_idle.parallel = task.Task.Task_parallel.ALL
        tasks_after_get_drink.add(task_left_arm_idle)
        
        #  将右臂(拿水)抬到指定位置(可并行，固定)
        task_right_arm_water_delivery = task.Task_manipulation(task.Task_type.Task_manipulation.Move,None,utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_delivery)
        task_right_arm_water_delivery.parallel = task.Task.Task_parallel.ALL
        tasks_after_get_drink.add(task_right_arm_water_delivery)
        
        #  机器人后退(可前并行，固定)
        task_rotation = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,back_meters=0.4)
        task_rotation.parallel = task.Task.Task_parallel.ALL
        tasks_after_get_drink.add(task_rotation)
        
        #  导航前往n号桌(不可并行，半动态)
        if table_id == utilis.Device_id.LEFT:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_left_service_desk,None,system.anchor_point.map_left_service_desk)
        elif table_id == utilis.Device_id.RIGHT:
            task_navigation_to_service_desk = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_right_service_desk,None,system.anchor_point.map_right_service_desk)
        else:
            raise ValueError("Invalid table_id")
        tasks_after_get_drink.add(task_navigation_to_service_desk)
        
        #  将饮料臂放到指定位置后松开(不可并行，固定)
        task_right_arm_placement_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_cup,None,utilis.Device_id.RIGHT,\
            system.anchor_point.right_arm_cup_placement,robot.manipulation_status.clamp.status.OPEN)
        tasks_after_get_drink.add(task_right_arm_placement_cup)
        
        #  将左,右臂放到空闲位置(可并行，固定)
        task_arm_idle   = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE,None,utilis.Device_id.LEFT_RIGHT,\
                [system.anchor_point.right_arm_idle,system.anchor_point.right_arm_idle],\
                [robot.manipulation_status.clamp.status.OPEN,robot.manipulation_status.clamp.status.OPEN])
        task_arm_idle.parallel = task.Task.Task_parallel.ALL
        tasks_after_get_drink.add(task_arm_idle)
        
        #  机器人后退(可前并行，固定)
        task_rotation2 = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,back_meters=0.4)
        task_rotation2.parallel = task.Task.Task_parallel.ALL
        tasks_after_get_drink.add(task_rotation2)
        
        # 赋值
        return tasks_after_get_drink
        
        
    # 创建前往初始点的任务
    def create_task_navigate_to_init_point(self):
        #  前往初始位置
        return task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,None)

    # 创建把左、右臂放到空闲位置的任务
    def create_task_arms_to_idle(self):
        #  将左、右臂抬到空闲位置(不可并行，共用)
        task_arm_idle   = task.Task_manipulation(task.Task_type.Task_manipulation.Move,None,utilis.Device_id.LEFT_RIGHT,\
                [system.constant_config.arm_anchor_point.right_arm_idle,system.constant_config.arm_anchor_point.right_arm_idle],\
                [robot.manipulation_status.clamp.status.OPEN,robot.manipulation_status.clamp.status.OPEN])
        task_arm_idle.parallel = task.Task.Task_parallel.ALL
        return task_arm_idle
    
    # 创建抓取零食的任务
    def create_task_grasp_snack(self,image_rec_task:task.Task_image_rec):
        # 抓取零食任务序列
        task_grasp_snack_seq = task.Task_sequence()
        
        # 抓取零食
        task_grasp_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_snack,None,utilis.Device_id.TBD,\
            target_clamps_status=robot.manipulation_status.clamp.status.CLOSE)
        task_grasp_snack.parallel = task.Task.Task_parallel.ALL
        task_grasp_snack.status = task.Task.Task_status.NOTREADY       # 需要参数
        task_grasp_snack_seq.add(task_grasp_snack)
        
        # 放置零食
        task_placement_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_snack,None,utilis.Device_id.TBD,\
            target_clamps_status=robot.manipulation_status.clamp.status.OPEN)
        task_placement_snack.parallel = task.Task.Task_parallel.ALL
        task_placement_snack.status = task.Task.Task_status.NOTREADY   # 需要参数
        task_placement_snack.add_predecessor_task(task_grasp_snack)    # 添加前置任务
        task_grasp_snack_seq.add(task_placement_snack)
        
        # 将抓取零食任务与图像识别任务绑定
        image_rec_task.add_need_modify_task(task_grasp_snack) 
        image_rec_task.add_need_modify_task(task_placement_snack)
        
        return task_grasp_snack_seq
    

# TODO:待完成
# 图像识别驱动的任务安排
class Image_rec_driven_task_schedul():
    def __init__(self,task_manager:Task_manager):
        self.task_manager = task_manager
    
    # 更新任务参数
    # TODO:待更新
    def updata_task_params(self,current_task:task.Task_image_rec,image_rec_result:msg.ImageRecResult):
        if current_task.task_type == task.Task_type.Task_image_rec.SNACK:
            snack_positions = image_rec_result.obj_positions
            for i in range (current_task.get_need_modify_task_count()/2):
                grasp_snack :task.Task_manipulation  = current_task.need_modify_tasks[i]
                lossen_snack:task.Task_manipulation  = current_task.need_modify_tasks[i + 1]
                
                grasp_snack.status  = task.Task.Task_status.BEREADY
                lossen_snack.status = task.Task.Task_status.BEREADY
        elif current_task.task_type == task.Task_type.Task_image_rec.CUP_COFFEE_MACHINE:
            cup_position       = image_rec_result.obj_positions[0]   # 杯子位置
            water_cup_position = image_rec_result.obj_positions[1]   # 装咖啡位置
            
            grasp_cup:task.Task_manipulation  = current_task.get_need_modify_task(0)
            move_cup :task.Task_manipulation  = current_task.get_need_modify_task(1)
            
            grasp_cup.status   = task.Task.Task_status.BEREADY
            move_cup.status    = task.Task.Task_status.BEREADY
            
        elif current_task.task_type == task.Task_type.Task_image_rec.COFFEE_MACHIE_SWITCH:
            move_under_switch:task.Task_manipulation   = current_task.get_need_modify_task(0)
            turn_on_switch   :task.Task_manipulation   = current_task.get_need_modify_task(1)
            move_over_switch :task.Task_manipulation   = current_task.get_need_modify_task(2)
            turn_off_switch  :task.Task_manipulation   = current_task.get_need_modify_task(3)
            
            move_under_switch.status = task.Task.Task_status.BEREADY
            turn_on_switch.status    = task.Task.Task_status.BEREADY
            move_over_switch.status  = task.Task.Task_status.BEREADY
            turn_off_switch.status   = task.Task.Task_status.BEREADY
            



def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('assign_tasks')
    global system
    system = System()

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("assign_tasks")
        # 按照设定的频率延时
        rate.sleep()
    # rospy.spin()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
