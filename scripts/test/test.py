#! /usr/bin/env /python3

"""
    Python 版 HelloWorld

"""
import rospy
import cv2
import run_task.msg as msg




class Hello:
    current = None
    def __init__(self,name):
        rospy.loginfo("Hello World!!!!")
        self.name = name
    
    def print_name(self):
        rospy.loginfo(f"Hello World!!!! {self.name}")
        
        
    def fn(self):
        self.current.print_name()
        
    

if __name__ == "__main__":
    rospy.init_node("Hello")
    # hello = Hello("JNU")
    # Hello.current = hello
    # hello.print_name()
    
    # hello2 = Hello("JNU2")
    # Hello.current = hello2
    # hello2.print_name()
    # print(cv2.__version__)
    # a = msg.MoveArmGoal()
    # print(isinstance(a.arm_pose, msg.ArmPose))
        # 设置发布消息的频率，1Hz
    angles = rospy.get_param('~CupRec/angles', [])

    print("Angles:", angles)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("arm")
        # 按照设定的频率延时
        rate.sleep()
    
