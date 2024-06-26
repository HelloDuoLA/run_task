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
    target_pose = utilis.Pose2D(-1.5,6.0+2,0.68)
    # target_pose = utilis.Pose2D(-1.5,6.0,0.68)
    task_navi = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,finish_call_back,target_pose)
    navigation_actuator = assign_tasks.navigation_actuator()
    navigation_actuator.run(task_navi)
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("test_navigation")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
