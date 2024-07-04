#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import actionlib
import random
from pymycobot import Mercury
from enum import Enum,auto # 任务字典

# 增加头文件路径 
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import run_task.msg as msg
import run_task.srv as srv
import robot


class PoseType(Enum):
    ANGLE       = 0           # 已经打开
    BASE_COORDS = auto()
    
    def __str__(self) -> str:
        return self.name
    
    def __eq__(self, value: object) -> bool:
        if isinstance(value, self.__class__):
            return self.value == value.value
        elif isinstance(value, int):
            return self.value == value

# 机械臂控制器
class arm_controller():
    def __init__(self,id:utilis.Device_id) -> None:
        self.id = id 
        if(self.id == utilis.Device_id.LEFT):
            self.action_name      = utilis.Topic_name.left_arm_action            # action名称
            self.pub_pose_topic   = utilis.Topic_name.left_arm_pose              # 发布机械臂状态
            self.control_instance = Mercury("/dev/left_arm") 
            self.arm_name = "left_arm"             
        elif(self.id == utilis.Device_id.RIGHT):
            self.action_name      = utilis.Topic_name.right_arm_action           # action名称
            self.pub_pose_topic   = utilis.Topic_name.right_arm_pose             # 发布机械臂状态
            self.control_instance = Mercury("/dev/right_arm")                    # TODO需要修改 驱动句柄
            self.arm_name = "right_arm"     
        
        power_on = self.control_instance.is_power_on() 
        if power_on :
            rospy.loginfo(f"{self.arm_name} is power on")
        else:
            rospy.loginfo(f"{self.arm_name} is not power on ret:{power_on} ")

        
        self.action = self.arm_action(self.action_name,self.control_instance,id)
        self.action.start_action()
    
    # 机械臂 action 
    class arm_action():
        def __init__(self,action_name,control_instance, id) -> None:
            self.id               = id
            self.action_server    = actionlib.SimpleActionServer(action_name, msg.MoveArmAction, self.execute_cb, False)
            self.control_instance = control_instance
            rospy.loginfo(f"node: {rospy.get_name()}, init {action_name} arm action server")
        
        # 启动action
        def start_action(self):    
            self.action_server.start()
        # 执行
        def execute_cb(self, goal:msg.MoveArmGoal):
            rospy.loginfo(f"node: {rospy.get_name()}, arm action server execute. goal: {goal}")
            # 1. 解析目标值
            goal_arm_pose     = goal.arm_pose
            goal_grasp_speed  = goal.grasp_speed
            goal_grasp_first  = goal.grasp_first
            goal_grasp_flag   = goal.grasp_flag
            change_grasp      = False # 是否改变夹具状态
            
            # if goal_grasp_flag != robot.manipulation_status.clamp.status.DONTCANGE:
            #     change_grasp = True
            
            # if change_grasp == True and goal_grasp_first == True:
            #     rospy.loginfo(f"node: {rospy.get_name()}, goal_grasp_flag: {goal_grasp_flag}")
            #     # 夹具动作
            #     if goal_grasp_flag == robot.manipulation_status.clamp.status.OPEN:
            #         self.grab_release(goal_grasp_speed)       # 松开
            #     elif goal_grasp_flag == robot.manipulation_status.clamp.status.CLOSE:
            #         self.grab_grip(goal_grasp_speed)          # 抓紧
            
            
            # 2. 给机械臂发送目标值
            self.move_arm(goal_arm_pose.type_id,goal_arm_pose.arm_pose)
            
            # 3. 发送连续反馈
            # rate = rospy.Rate(10) 
            # if result:
            self.action_server.set_succeeded() #可以添加结果参数
            # else:
            #     self.action_server.set_aborted()   #TODO: 失败是否是这样的
    
        def move_arm(self,pose_type:PoseType,pose_list):
            rospy.loginfo(f"{self.id} arm move to {pose_list} using {pose_type}")
            if pose_type == PoseType.ANGLE:
                return self.control_instance.send_angles(pose_list,50)
            elif pose_type == PoseType.BASE_COORDS:
                return self.control_instance.send_base_coords(pose_list,50)
            

    
def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('arm_node')

    arm_name   = rospy.get_param(f'~arm_name')
    global arm_controller
    if arm_name == "Left":
        arm_controller  = arm_controller(utilis.Device_id.LEFT)
    elif arm_name == "Right":
        arm_controller = arm_controller(utilis.Device_id.RIGHT)
    
    server = rospy.Service(f"Check{arm_name}ArmPose",srv.CheckArmPose,doCheckArmPose)
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    
    while not rospy.is_shutdown():
        rospy.loginfo(f"{arm_name}_arm_node")
        rate.sleep()

# 服务函数 
def doCheckArmPose(req:srv.CheckArmPoseRequest):
    rospy.loginfo(f"node: {rospy.get_name()}, doCheckArmPose. req: {req}")
    resp = srv.CheckArmPoseResponse()
    resp.type_id = req.type_id
    if req.type_id == 0:
        resp.arm_pose = arm_controller.control_instance.get_base_coords()
    elif req.type_id == 1:
        resp.arm_pose = arm_controller.control_instance.get_angles()

    rospy.loginfo(f"node: {rospy.get_name()}, doCheckArmPose. req: {req} resp arm : {resp.arm_pose}")
    return resp

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
