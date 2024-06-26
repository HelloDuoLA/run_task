#! /usr/bin/env /usr/bin/python3

"""
    Python 版 HelloWorld

"""
import rospy
import cv2
# from run_task.msg import ObjPositionWithID
# from run_task.srv import ImageRec,ImageRecRequest,ImageRecResponse
import run_task.srv as srv
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
    a = msg.MoveArmGoal()
    print(isinstance(a.arm_pose, msg.ArmPose))
    
