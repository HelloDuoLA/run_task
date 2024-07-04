#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import run_task.msg as msg


rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import task




def test():
    ac = actionlib.SimpleActionClient('right_arm_action', msg.MoveArmAction)
    ac.wait_for_server()
    goal = msg.MoveArmGoal()
    goal.arm_pose.arm_pose = [66.27, 5.44, -148.83, -123.8, 72.58, -24.6]
    goal.arm_pose.type_id = 0
    ac.send_goal(goal,navigation_task_done_callback,navigation_task_active_callback,navigation_task_feedback_callback)
    
def navigation_task_done_callback(status, result):
    rospy.loginfo(f"node: {rospy.get_name()}, navigation done. status:{status} result:{result}")
    
# 激活回调
def navigation_task_active_callback():
    rospy.loginfo(f"node: {rospy.get_name()}, navigation active")

# 反馈回调
def navigation_task_feedback_callback(feedback):
    rospy.loginfo(f"node: {rospy.get_name()},  feedback")

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('test_move_arm')
    rate = rospy.Rate(1)
    test()
    while not rospy.is_shutdown():
        rospy.loginfo("test_move_arm'")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
