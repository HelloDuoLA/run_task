#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import actionlib
import random
from pymycobot import Mercury
from enum import Enum,auto # 任务字典
import time

# 增加头文件路径 
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import run_task.msg as msg
import run_task.srv as srv



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
        
# 机械臂位姿
class Arm_pose():
    # 初始化
    def __init__(self,arm_pose:msg.ArmPoseWithID=[-999,-999,-999,-999,-999,-999],type_id=PoseType.BASE_COORDS,arm_id=utilis.Device_id.TBD):
        if isinstance(arm_pose, msg.ArmPose):
            self.arm_pose = arm_pose.arm_pose
            self.type_id  = arm_pose.type_id
            self.arm_id   = arm_id
        elif isinstance(arm_pose,msg.ArmPoseWithID):
            self.arm_pose = arm_pose.arm_pose
            self.type_id  = arm_pose.type_id
            self.arm_id   = arm_pose.arm_id
        elif isinstance(arm_pose, list) and len(arm_pose) == 6:
            self.arm_pose = arm_pose
            self.type_id  = type_id
            self.arm_id   = arm_id
        else:
            raise ValueError("Invalid initialization parameter for arm_pose")
    
    def set_joint_angle(self,joint_index,angle):
        self.arm_pose[joint_index] = angle
    
    def set_base_coords_x(self,x):
        self.arm_pose[0] = x
        
    def set_base_coords_y(self,y):
        self.arm_pose[1] = y
        
    def set_base_coords_z(self,z):
        self.arm_pose[2] = z
    
    def set_base_coords_rx(self,rx):
        self.arm_pose[3] = rx
    
    def set_base_coords_ry(self,ry):
        self.arm_pose[4] = ry
    
    def set_base_coords_rz(self,rz):
        self.arm_pose[5] = rz
        
    def set_id(self,arm_id:utilis.Device_id):
        self.arm_id = arm_id

    # 重写等号
    def __eq__(self, other):
        if isinstance(other, Arm_pose):
            return (self.arm_pose == other.arm_pose)
        elif isinstance(other, list):
            return (self.arm_pose == other)
    # 重写打印输出
    def __str__(self):
        if self.type_id == PoseType.ANGLE:
            return f"arm {self.arm_id} joint:{self.arm_pose}"
        elif self.type_id == PoseType.BASE_COORDS:
            return f"arm {self.arm_id} coords:{self.arm_pose}"
    # 将列表状态输出为action的数据结构
    def list_to_msg(self,arm_list_status:list):
        arm_pose = msg.ArmPose()
        arm_pose.arm_pose = self.arm_pose
        arm_pose.type_id  = self.type_id
        return arm_pose


# 机械臂控制器
class Arm_controller():
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
        
        rospy.loginfo(f"{self.arm_name} is power status {power_on}")
        while not power_on:
            self.control_instance.power_on()
            power_on = self.control_instance.is_power_on()

        
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
            
            result = msg.MoveArmResult()
            result.arm_id    = goal.arm_id
            result.task_index = goal.task_index
            # 3. 发送连续反馈
            # rate = rospy.Rate(10) 
            # if result:
            self.action_server.set_succeeded(result) #可以添加结果参数
            # else:
            #     self.action_server.set_aborted()   #TODO: 失败是否是这样的
    
        def move_arm(self,pose_type:PoseType,pose_list):
            rospy.loginfo(f"{self.id} arm move to {pose_list} using {pose_type}")
            if pose_type == PoseType.ANGLE:
                result = self.control_instance.send_angles(pose_list,50)
                self.wait()
                return result 
            elif pose_type == PoseType.BASE_COORDS:
                result = self.control_instance.send_base_coords(pose_list,50)
                self.wait()
                return result 
        
        def wait(self):
            time.sleep(0.3)
            while(self.control_instance.is_moving()):
                # print("arm is moving")
                time.sleep(0.03)
            

    
def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('arm_node')

    arm_name   = rospy.get_param(f'~arm_name')
    global arm_controller
    if arm_name == "Left":
        arm_controller  = Arm_controller(utilis.Device_id.LEFT)
    elif arm_name == "Right":
        arm_controller = Arm_controller(utilis.Device_id.RIGHT)
    
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
    if req.type_id == PoseType.ANGLE:
        resp.arm_pose = arm_controller.control_instance.get_angles()
    elif req.type_id == PoseType.BASE_COORDS:
        resp.arm_pose = arm_controller.control_instance.get_base_coords()

    rospy.loginfo(f"node: {rospy.get_name()}, doCheckArmPose. req: {req} resp arm : {resp.arm_pose}")
    return resp

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
