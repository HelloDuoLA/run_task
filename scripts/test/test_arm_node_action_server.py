#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis

from std_msgs.msg import String
from run_task.msg import *
import actionlib


def done_cb(state,result):
    rospy.loginfo("响应结果:%d",result.result)
    rospy.loginfo("状态:%s",state)

def active_cb():
    rospy.loginfo("服务被激活....")


def fb_cb(fb):
    rospy.loginfo(f"fb {fb}")
    
def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('template')

    # 3.创建 action Client 对象
    client = actionlib.SimpleActionClient("right_arm_action",arm_actionAction)
    # 4.等待服务
    client.wait_for_server()
    # 5.组织目标对象并发送
    goal_obj = utilis.arm_status.list_to_action([1,2,3,4,5,6],arm_actionGoal)
    client.send_goal(goal_obj,done_cb,active_cb,fb_cb)
    rospy.sleep(2)  # 等待5秒
    client.cancel_goal()  # 取消当前目标
    
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
