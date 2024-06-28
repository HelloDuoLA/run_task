#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import assign_tasks
import task
import utilis

from std_msgs.msg import String

def finish_call_back(status, result):
    rospy.loginfo(f"finish_call_back status : {status}")
    
def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('test_navigation')
    
    pose_snack          =  assign_tasks.System._constant_config_to_robot_anchor_pose_orientation("SnackDesk")
    pose_drink          =  assign_tasks.System._constant_config_to_robot_anchor_pose_orientation("DrinkDesk")
    pose_right_desk     =  assign_tasks.System._constant_config_to_robot_anchor_pose_orientation("RightServiceDesk")
    pose_left_desk      =  assign_tasks.System._constant_config_to_robot_anchor_pose_orientation("LeftServiceDesk")
    
    
    task_navi1 = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,finish_call_back,pose_snack)
    task_navi2 = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,finish_call_back,pose_drink)
    task_navi3 = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,finish_call_back,pose_right_desk)
    task_navi4 = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,finish_call_back,pose_left_desk)
    
    navigation_actuator = assign_tasks.Navigation_actuator()
    navigation_actuator.run(task_navi1)
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("test_mercury_navigation")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
