#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import actionlib
from enum import Enum,auto
from geometry_msgs.msg import Twist
import time

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


# 控制指令
class Control_cmd(Enum):
    MOVEBACK = 0
    
    def __str__(self) -> str:
        return super().__str__()
    
    def __eq__(self, value: object) -> bool:
        if isinstance(value, self.__class__):
            return self.value == value.value
        elif isinstance(value, int):
            return self.value == value

# 控制指令action -> msg
class ControlCmdActionServer:
    def __init__(self):
        #SimpleActionServer(name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer(utilis.Topic_name.control_cmd_action,msg.ControlCmdAction,self.cb,False)
        self.server.start()
        rospy.loginfo(f"{rospy.get_name()}server start")
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        

    def cb(self,goal:msg.ControlCmdGoal):
        rospy.loginfo(f"{rospy.get_name()} get move back goal {goal}")
        if goal.operation == Control_cmd.MOVEBACK:
            move_cmd = Twist()
            move_cmd.linear.x = -goal.speed # 设置线速度，负值表示后退
            move_cmd.angular.z = 0 # 保持直线行驶
            # 计算后退的时间
            duration = goal.meters / goal.speed
            rate = rospy.Rate(10) # 10hz
            start_time = time.time()
            while time.time() - start_time < duration:
                self.pub.publish(move_cmd)
                rate.sleep()

            # 停止小车
            move_cmd.linear.x = 0
            self.pub.publish(move_cmd)
            
            # 发送返回值
            result = msg.ControlCmdResult()
            result.task_index = goal.task_index
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted()
            # raise ValueError("Invalid operation")   
        
        #1.解析目标值

        

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('control_cmd')

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(10)
    
    ControlCmdActionServer()

    while not rospy.is_shutdown():
        rospy.loginfo("control_cmd")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
