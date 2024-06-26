'''
Description: 
Version: 1.0
Author: xzc
Date: 2024-06-03 12:46:59
'''
#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

# 本科生操作手册1.0
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class MapNavigation:
    def __init__(self):
        self.goalReached = None
        rospy.init_node('map_navigation', anonymous=False)
    # move_base
    def moveToGoal(self, xGoal, yGoal, orientation_z, orientation_w):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        while (not ac.wait_for_server(rospy.Duration.from_sec(5.0))):sys.exit(0)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = orientation_z
        goal.target_pose.pose.orientation.w = orientation_w
        rospy.loginfo("Sending goal location ...")
        
        ac.send_goal(goal)
        ac.wait_for_result(rospy.Duration(600))
        if (ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
        return False
    

if __name__ == "__main__":
    goal_1 = [(-0.0277661, -0.00824622, 0.0431145, 0.999068)]
    goal_2 = [(0.428357, -1.99509, 0.999547, -0.037365)]
    goal_3 = [(0.318357, -2.10509, -0.681143, 0.732115)]
    goal_4 = [(1.88323, -1.84847, 0.0746518, 0.997171)]
    goal_5 = [(2.05847, -0.492321, -0.00194264, 0.999828)]
    map_navigation = MapNavigation()
    x_goal, y_goal, orientation_z, orientation_w = goal_2[0]
    flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal,orientation_z, orientation_w)
    if flag_feed_goalReached:
        print("command completed")
    else:
        raise ValueError