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

can_run_task = True
# 让小车在5点之间进行巡航

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('template')

    init_pose    = constant_config_to_robot_anchor_pose_orientation("InitialPose")
    middle_point = constant_config_to_robot_anchor_pose_orientation("MiddlePoint")
    SnackDesk    = constant_config_to_robot_anchor_pose_orientation("SnackDesk")
    DrinkDesk    = constant_config_to_robot_anchor_pose_orientation("DrinkDesk")
    RightServiceDesk = constant_config_to_robot_anchor_pose_orientation("RightServiceDesk")
    LeftServiceDesk  = constant_config_to_robot_anchor_pose_orientation("LeftServiceDesk")
    
    task_init_pose         = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point, None, init_pose)
    task_SnackDesk         = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_snack_desk, None, SnackDesk)
    task_snack_desk_back   = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,back_meters=0.6)
    task_DrinkDesk         = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_drink_desk, None, DrinkDesk)
    task_drink_desk_back   = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,back_meters=0.6)
    task_RightServiceDesk  = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_right_service_desk, None, RightServiceDesk)
    task_right_service_back= task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,back_meters=0.2)
    task_LeftServiceDesk   = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_left_service_desk, None, LeftServiceDesk)
    task_left_service_back = task.Task_navigation(task.Task_type.Task_navigate.Move_backward,None,back_meters=0.2)
    
    task_middle_point1     = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_middle_point,   None, middle_point)
    task_middle_point2     = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_middle_point,   None, middle_point)
    
    rospy.loginfo(f"init_pose        : {init_pose}")
    rospy.loginfo(f"SnackDesk        : {SnackDesk}")
    rospy.loginfo(f"DrinkDesk        : {DrinkDesk}")
    rospy.loginfo(f"RightServiceDesk : {RightServiceDesk}")
    rospy.loginfo(f"LeftServiceDesk  : {LeftServiceDesk}")

    # task_list = [task_SnackDesk ,task_RightServiceDesk, task_DrinkDesk ,task_LeftServiceDesk,task_init_pose]
    # task_list = [task_move_back1]
    task_list = []
    # task_list.append(task_middle_point1)
    task_list.append(task_SnackDesk)
    task_list.append(task_snack_desk_back)
    task_list.append(task_RightServiceDesk)
    task_list.append(task_right_service_back)
    # task_list.append(task_middle_point2)
    task_list.append(task_DrinkDesk)
    task_list.append(task_drink_desk_back)
    task_list.append(task_LeftServiceDesk)
    task_list.append(task_left_service_back)
    task_list.append(task_init_pose)
    
    
    navigation_actuator = Navigation_actuator()
    
    # test()  # 单独前往某一节点
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)
    task_index = 0
    global can_run_task
    while not rospy.is_shutdown():
        if can_run_task  and  task_index < len(task_list):
            can_run_task = False
            navigation_actuator.run(task_list[task_index])
            task_index += 1

        rospy.loginfo("crui")
        # 按照设定的频率延时
        rate.sleep()
        
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
        rospy.loginfo("waiting for move_base, control cmd server")
        # TODO:调试需要,暂时注释
        self.move_base_ac.wait_for_server()
        self.control_cmd_ac.wait_for_server()
        rospy.loginfo("move_base, control cmd server start")
        
        self.running_tasks_manager = task.Task_manager_in_running() # 正在执行的任务管理器
        
    # 运行
    def run(self, navigation_task:task.Task_navigation):
        task_index = self.running_tasks_manager.add_task(navigation_task)
        navigation_task.update_start_status() # 刷新开始时间
        
        # 后退任务
        if navigation_task.task_type == task.Task_type.Task_navigate.Move_backward:
            goal = msg.ControlCmdGoal()
            goal.task_index = task_index
            goal.operation  = control_cmd.Control_cmd.MOVEBACK.value
            goal.speed      = navigation_task.move_back_speed
            goal.meters     = navigation_task.back_meters
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
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"node: {rospy.get_name()}, navigation succeed. status : {status}")
        else:
            rospy.loginfo(f"node: {rospy.get_name()}, navigation failed. status : {status}")
        global can_run_task 
        can_run_task = True
    
    # 激活回调
    @staticmethod
    def navigation_task_active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, navigation active")

    # 反馈回调
    @staticmethod
    def navigation_task_feedback_callback(feedback:MoveBaseFeedback):
        pose = feedback.base_position.pose
        pose3D = utilis.Pose3D.instantiate_by_geometry_msg(pose)
        rospy.loginfo(f"node: {rospy.get_name()}, navigation feedback. pose:x = {pose3D.x} y = {pose3D.y} yaw = {pose3D.yaw}")

    # 直接控制完成回调
    @staticmethod
    def control_cmd_task_done_callback(status, result:msg.ControlCmdResult):
        rospy.loginfo(f"node: {rospy.get_name()}, navigation done. status:{status} result:{result}")
        global can_run_task 
        can_run_task = True
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"node: {rospy.get_name()}, navigation succeed. status : {status}")
        else:
            rospy.loginfo(f"node: {rospy.get_name()}, navigation failed. status : {status}")
    
    # 激活回调
    @staticmethod
    def control_cmd_active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, control cmd active")

    # 反馈回调
    @staticmethod
    def control_cmd_feedback_callback(feedback:MoveBaseFeedback):
        rospy.loginfo(f"node: {rospy.get_name()}, control cmd feedback. feedback = {feedback}")
        

def test():
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    ac.wait_for_server()
    anchor_point_name = "InitialPose"
    # anchor_point_name = "DrinkDesk"
    # anchor_point_name = "SnackDesk"
    # anchor_point_name = "RightServiceDesk"
    # anchor_point_name = "LeftServiceDesk"
    x   = rospy.get_param(f'~{anchor_point_name}/position_x')
    y   = rospy.get_param(f'~{anchor_point_name}/position_y')
    z   = rospy.get_param(f'~{anchor_point_name}/position_z')
    o_x = rospy.get_param(f'~{anchor_point_name}/orientation_x')
    o_y = rospy.get_param(f'~{anchor_point_name}/orientation_y')
    o_z = rospy.get_param(f'~{anchor_point_name}/orientation_z')
    o_w = rospy.get_param(f'~{anchor_point_name}/orientation_w')
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp    = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    # orientation = quaternion_from_euler(0, 0, self.task.target_3D_pose.yaw)
    goal.target_pose.pose.orientation.x = o_x
    goal.target_pose.pose.orientation.y = o_y
    goal.target_pose.pose.orientation.z = o_z
    goal.target_pose.pose.orientation.w = o_w
    rospy.loginfo(f"navigation to position {goal.target_pose.pose.position}  orientation {goal.target_pose.pose.orientation}")
    ac.send_goal(goal,navigation_task_done_callback,navigation_task_active_callback,navigation_task_feedback_callback)
    
def navigation_task_done_callback(status, result):
    rospy.loginfo(f"node: {rospy.get_name()}, navigation done. status:{status} result:{result}")
    
# 激活回调
def navigation_task_active_callback():
    rospy.loginfo(f"node: {rospy.get_name()}, navigation active")

# 反馈回调
def navigation_task_feedback_callback(feedback:MoveBaseFeedback):
    pose = feedback.base_position.pose
    pose3D = utilis.Pose3D.instantiate_by_geometry_msg(pose)
    rospy.loginfo(f"node: {rospy.get_name()}, navigation feedback. pose:x = {pose3D.x} y = {pose3D.y} yaw = {pose3D.yaw}")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
