#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from tf.transformations import quaternion_from_euler, euler_from_quaternion


rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import task
import run_task.msg as msg
import control_cmd

import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import tf_conversions  # 用于四元数和欧拉角的转换

current_task = None

# 让小车在5点之间进行巡航

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('template')


    init_pose        = constant_config_to_robot_anchor_pose_orientation("InitialPose")
    SnackDesk        = constant_config_to_robot_anchor_pose_orientation("SnackDesk")
    DrinkDesk        = constant_config_to_robot_anchor_pose_orientation("DrinkDesk")
    RightServiceDesk = constant_config_to_robot_anchor_pose_orientation("RightServiceDesk")
    LeftServiceDesk  = constant_config_to_robot_anchor_pose_orientation("LeftServiceDesk")
    snack_deck_move_forward_pose =  _get_control_cmd_xy("SnackDeckMoveforward")
    drink_deck_move_forward_pose =  _get_control_cmd_xy("DrinkDeckMoveforward")
    left_deck_move_forward_pose  =  _get_control_cmd_xy("LeftDeckMoveforward")
    right_deck_move_forward_pose =  _get_control_cmd_xy("RightDeckMoveforward")
    
    snack_deck_move_back_pose =  _get_control_cmd_xy("SnackDeckMoveBack")
    drink_deck_move_back_pose =  _get_control_cmd_xy("DrinkDeckMoveBack")
    left_deck_move_back_pose  =  _get_control_cmd_xy("LeftDeckMoveBack")
    right_deck_move_back_pose =  _get_control_cmd_xy("RightDeckMoveBack")
            
    task_init_pose            = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point, None, init_pose)
    task_SnackDesk            = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_snack_desk, None, SnackDesk)
    task_snack_desk_back      = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,snack_deck_move_back_pose)
    task_snack_desk_forward   = task.Task_navigation(task.Task_type.Task_navigate.Move_forward,None,snack_deck_move_forward_pose)
    task_DrinkDesk            = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_drink_desk, None, DrinkDesk)
    task_drink_desk_back      = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,drink_deck_move_back_pose)
    task_drink_desk_forward   = task.Task_navigation(task.Task_type.Task_navigate.Move_forward,None,drink_deck_move_forward_pose)
    task_RightServiceDesk     = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_right_service_desk, None, RightServiceDesk)
    task_right_service_back   = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,right_deck_move_back_pose)
    task_right_service_forward= task.Task_navigation(task.Task_type.Task_navigate.Move_forward,None,right_deck_move_forward_pose)
    task_LeftServiceDesk      = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_left_service_desk, None, LeftServiceDesk)
    task_left_service_back    = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,left_deck_move_back_pose)
    task_left_service_forward = task.Task_navigation(task.Task_type.Task_navigate.Move_forward,None,left_deck_move_forward_pose)
    
    
    rospy.loginfo(f"init_pose        : {init_pose}")
    rospy.loginfo(f"SnackDesk        : {SnackDesk}")
    rospy.loginfo(f"DrinkDesk        : {DrinkDesk}")
    rospy.loginfo(f"RightServiceDesk : {RightServiceDesk}")
    rospy.loginfo(f"LeftServiceDesk  : {LeftServiceDesk}")

    five_point_task_list = []
    
    # # 跑五点
    # if path_num == 0:
    five_point_task_list.append(task_SnackDesk)
    five_point_task_list.append(task_snack_desk_back)
    five_point_task_list.append(task_RightServiceDesk)
    five_point_task_list.append(task_right_service_back)
    five_point_task_list.append(task_DrinkDesk)
    five_point_task_list.append(task_drink_desk_back)
    five_point_task_list.append(task_LeftServiceDesk)
    five_point_task_list.append(task_left_service_back)
    five_point_task_list.append(task_init_pose)

    
    navigation_actuator = Navigation_actuator()
    
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)
    task_index = 0
    global running
    
    # 各个状态位
    running = False # 是否正在执行任务
    running_five_point = False # 是否正在执行五点任务
    
    # 全局变量
    global current_task 
    
    while not rospy.is_shutdown():
        rospy.loginfo("cruise running")
        if running == False and running_five_point == False:
            rospy.loginfo("输入数字选择目的点")
            rospy.loginfo("0 : 五点寻航")
            rospy.loginfo("1 : 前往零食桌")
            rospy.loginfo("10: 零食桌后退")
            rospy.loginfo("11: 零食桌前进")
            rospy.loginfo("2 : 前往饮料桌")
            rospy.loginfo("20: 饮料桌后退")
            rospy.loginfo("21: 饮料桌前进")
            rospy.loginfo("3 : 前往左服务桌")
            rospy.loginfo("30: 左服务桌后退")
            rospy.loginfo("31: 左服务桌前进")
            rospy.loginfo("4 : 前往右服务桌")
            rospy.loginfo("40: 右服务桌后退")
            rospy.loginfo("41: 右服务桌前进")
            rospy.loginfo("5 : 前往初始点")
            
            # 获取用户输入
            user_input = input("请输入您的指令: ")
            
            if user_input == "0":
                task_index = 0
                
                running_five_point = True
            elif user_input == "1":
                current_task = task_SnackDesk
                navigation_actuator.run(task_SnackDesk)
                
            elif user_input == "10":
                current_task = task_snack_desk_back
                navigation_actuator.run(task_snack_desk_back)
            
            elif user_input == "11":
                current_task = task_snack_desk_forward
                navigation_actuator.run(task_snack_desk_forward)
                
            elif user_input == "2":
                current_task = task_DrinkDesk
                navigation_actuator.run(task_DrinkDesk)
                
            elif user_input == "20":
                current_task = task_drink_desk_back
                navigation_actuator.run(task_drink_desk_back)
            
            elif user_input == "21":
                current_task = task_drink_desk_forward
                navigation_actuator.run(task_drink_desk_forward)
                
            elif user_input == "3":
                current_task = task_LeftServiceDesk
                navigation_actuator.run(task_LeftServiceDesk)
                
            elif user_input == "30":
                current_task = task_left_service_back
                navigation_actuator.run(task_left_service_back)
            
            elif user_input == "31":
                current_task = task_left_service_forward
                navigation_actuator.run(task_left_service_forward)
                
            elif user_input == "4":
                current_task = task_RightServiceDesk
                navigation_actuator.run(task_RightServiceDesk)
                
            elif user_input == "40":
                current_task = task_right_service_back
                navigation_actuator.run(task_right_service_back)
            
            elif user_input == "41":
                current_task = task_right_service_forward
                navigation_actuator.run(task_right_service_forward)
                
            elif user_input == "5":
                current_task = task_init_pose
                navigation_actuator.run(task_init_pose)
                
            running    = True
            # 打印用户输入
            rospy.loginfo("您输入的指令是: %s", user_input)
        else:
            rospy.loginfo("任务正在执行中")
            if running_five_point == True:
                if running == False  and  task_index < len(five_point_task_list):
                    running = True
                    navigation_actuator.run(five_point_task_list[task_index])
                    task_index += 1
                elif running == False  and task_index == len(five_point_task_list):
                    running_five_point = False
                    rospy.loginfo("五点任务执行完毕")
            
        rate.sleep()
    

def _get_control_cmd_xy(name):
    target_pose = utilis.Pose3D()
    target_pose.x  = rospy.get_param(f'~{name}/x')
    target_pose.y  = rospy.get_param(f'~{name}/y')
    return target_pose


def constant_config_to_robot_anchor_pose_orientation(anchor_point_name):
    x   = rospy.get_param(f'~{anchor_point_name}/position_x')
    y   = rospy.get_param(f'~{anchor_point_name}/position_y')
    z   = rospy.get_param(f'~{anchor_point_name}/position_z')
    o_x = rospy.get_param(f'~{anchor_point_name}/orientation_x')
    o_y = rospy.get_param(f'~{anchor_point_name}/orientation_y')
    o_z = rospy.get_param(f'~{anchor_point_name}/orientation_z')
    o_w = rospy.get_param(f'~{anchor_point_name}/orientation_w')
    pose = utilis.Pose3D.instantiate_by_xyz_orientation(x,y,z,o_x,o_y,o_z,o_w)
    return pose

# 导航任务执行器
class Navigation_actuator():
    # 初始化
    def __init__(self):
        # 订阅导航Action   
        self.move_base_ac   = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.control_cmd_ac = actionlib.SimpleActionClient(utilis.Topic_name.control_cmd_action, msg.ControlCmdAction)
        rospy.loginfo("waiting for move_base")
        self.move_base_ac.wait_for_server()
        rospy.loginfo("waiting for control cmd server")
        self.control_cmd_ac.wait_for_server()
        
        
    # 运行
    def run(self, navigation_task:task.Task_navigation):
        navigation_task.update_start_status() # 刷新开始时间
        # 后退任务
        if navigation_task.task_type == task.Task_type.Task_navigate.Move_backward or \
            navigation_task.task_type == task.Task_type.Task_navigate.Move_forward:
            goal = msg.ControlCmdGoal()
            goal.operation  = control_cmd.Control_cmd.MOVEBACK.value
            goal.second     = navigation_task.move_back_second         # 后退秒数
            goal.x          = navigation_task.target_3D_pose.x         # 后退距离x
            goal.y          = navigation_task.target_3D_pose.y         # 后退距离y
            self.control_cmd_ac.send_goal(goal,self.control_cmd_task_done_callback,self.control_cmd_active_callback,self.control_cmd_feedback_callback)
        # 导航任务
        else:
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
        global running,current_task
        running = False
        x,y,yaw = get_transform_xy_yaw("map", "base_footprint")
        rospy.loginfo(f"node: {rospy.get_name()}, base_footprint to map :x = {x} y = {y} yaw = {yaw}")
        rospy.loginfo(f"node: {rospy.get_name()}, target error :x = {x - current_task.target_3D_pose.x}\
            y = {y - current_task.target_3D_pose.y} yaw = {yaw - current_task.target_3D_pose.yaw} ")
        
    # 激活回调
    @staticmethod
    def navigation_task_active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, navigation active")

    # 反馈回调
    @staticmethod
    def navigation_task_feedback_callback(feedback:MoveBaseFeedback):
        pose = feedback.base_position.pose
        pose3D = utilis.Pose3D.instantiate_by_geometry_msg(pose)
        # rospy.loginfo(f"node: {rospy.get_name()},1 navigation feedback. pose:x = {pose3D.x} y = {pose3D.y} yaw = {pose3D.yaw}")

    # 直接控制完成回调
    @staticmethod
    def control_cmd_task_done_callback(status, result:msg.ControlCmdResult):
        rospy.loginfo(f"node: {rospy.get_name()}, control cmd task done. status:{status} result:{result}")
        global running
        running = False
        x,y,yaw = get_transform_xy_yaw("map", "base_footprint")
        rospy.loginfo(f"node: {rospy.get_name()}, base_footprint to map :x = {x} y = {y} yaw = {yaw}")
    
    # 激活回调
    @staticmethod
    def control_cmd_active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, control cmd active")

    # 反馈回调
    @staticmethod
    def control_cmd_feedback_callback(feedback:MoveBaseFeedback):
        # rospy.loginfo(f"node: {rospy.get_name()}, control cmd feedback. feedback = {feedback}")
        pass     

def get_transform_xy_yaw(target_frame, source_frame):
    # rospy.init_node('transform_listener', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # 等待变换关系变得可用
        trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        # 提取xy位置
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        # 提取四元数
        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )
        # 四元数转欧拉角，提取yaw
        euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]  # Z轴旋转为yaw
        return x, y, yaw
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
        return None, None, None


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
