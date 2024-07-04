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

can_run_task = True
# 让小车在5点之间进行巡航

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('template')

    # init_pose = constant_config_to_robot_anchor_pose_orientation("InitialPose")
    # SnackDesk = constant_config_to_robot_anchor_pose_orientation("SnackDesk")
    # DrinkDesk = constant_config_to_robot_anchor_pose_orientation("DrinkDesk")
    # RightServiceDesk = constant_config_to_robot_anchor_pose_orientation("RightServiceDesk")
    # LeftServiceDesk  = constant_config_to_robot_anchor_pose_orientation("LeftServiceDesk")
    
    # task_init_pose         = task.Task_navigation(init_pose)
    # task_SnackDesk         = task.Task_navigation(SnackDesk)
    # task_DrinkDesk         = task.Task_navigation(DrinkDesk)
    # task_RightServiceDesk  = task.Task_navigation(RightServiceDesk)
    # task_LeftServiceDesk   = task.Task_navigation(LeftServiceDesk)
    
    # rospy.loginfo(f"init_pose        : {init_pose}")
    # rospy.loginfo(f"SnackDesk        : {SnackDesk}")
    # rospy.loginfo(f"DrinkDesk        : {DrinkDesk}")
    # rospy.loginfo(f"RightServiceDesk : {RightServiceDesk}")
    # rospy.loginfo(f"LeftServiceDesk  : {LeftServiceDesk}")

    # task_list = [task_SnackDesk ,task_RightServiceDesk, task_DrinkDesk ,task_LeftServiceDesk,task_init_pose]
    # navigation_actuator = Navigation_actuator()
    
    test()
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)
    task_index = 0
    global can_run_task
    while not rospy.is_shutdown():
        # if can_run_task  and  task_index < len(task_list):
        #     can_run_task = False
        #     navigation_actuator.run(task_list[task_index])
        #     task_index += 1
            
            
        rospy.loginfo("arm")
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

class Navigation_actuator():
    # 初始化
    def __init__(self):
        # 订阅导航Action   
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("waitting for move_base server")
        self.ac.wait_for_server()

    # 运行
    def run(self, navigation_task:task.Task_navigation):
        self.task = navigation_task
        rospy.loginfo(f"navigation to {navigation_task.target_3D_pose}")
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
        rospy.loginfo(f"navigation to position {goal.target_pose.pose.position}  orientation {goal.target_pose.pose.orientation}")
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
        global system
        pose = feedback.base_position.pose
        pose3D = utilis.Pose3D.instantiate_by_geometry_msg(pose)
        rospy.loginfo(f"node: {rospy.get_name()}, navigation feedback. pose:x = {pose3D.x} y = {pose3D.y} yaw = {pose3D.yaw}")
        

def test():
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    ac.wait_for_server()
    # anchor_point_name = "InitialPose"
    # anchor_point_name = "DrinkDesk"
    anchor_point_name = "SnackDesk"
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
