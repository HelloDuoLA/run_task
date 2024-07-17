import rospy
import os
import sys
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import actionlib
import utilis
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from enum import Enum,auto # 任务字典
import run_task.msg as msg
import arm

# 机械臂状态类
class manipulation_status():
    # 初始化
    def __init__(self,id:utilis.Device_id):
        self.id          = id                # 机械臂编号
        self.arm_info    = self.arm()        # 机械臂状态
        self.camera_info = self.camera()     # 摄像头信息
        
    # 机械臂状态
    # ?有代码引用嘛
    class arm():
        class status(Enum):
            IDLE   = 0            # 空闲
            MOVING = auto()       # 正在移动
            BUSY   = auto()       # 忙碌
            
            def __str__(self) -> str:
                return self.name
        def __init__(self,arm_pose:arm.Arm_pose=arm.Arm_pose()):
            self.arm_status   = self.status.IDLE    # 机械臂状态,默认空闲
            self.arm_pose = arm_pose                # 机械臂位置
    
    # 摄像头类
    class camera():
        class status(Enum):
            IDLE = 0
            RUNNING = auto()
        
        def __init__(self):
            self.status = self.status.IDLE  # 摄像头状态,默认空闲
            
# 机器人类
class Robot():
    class Robot_status(Enum):
        IDLE    = 0           # 空闲
        MOVING  = auto()      # 移动中
        WORKING = auto()      # 工作中
        
        def __str__(self) -> str:
            return self.name
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value, int):
                return self.value == value
        
    class Common_status(Enum):
        IDLE      = 0           # 空闲
        BUSY      = auto()      # 忙碌
        DONCHANGE = auto()      # 不改变
        
        def __str__(self) -> str:
            return self.name
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value, int):
                return self.value == value
    
    def __str__(self):
        return f"Robot status: {self.robot_status}, left arm status: {self.left_manipulation_status.arm_info.arm_status}, \
            right arm status: {self.right_manipulation_status.arm_info.arm_status}"
    
    def __init__(self,pose:utilis.Pose3D=utilis.Pose3D()) -> None:
        self.robot_status              = self.Robot_status.IDLE                         # 机器人状态
        self.right_manipulation_status = manipulation_status(utilis.Device_id.RIGHT)    # 右臂
        self.left_manipulation_status  = manipulation_status(utilis.Device_id.LEFT)     # 左臂
        self.lossen_snack_point_status = self.Common_status.IDLE                        # 松开零食点状态

    

    # 获取2D位姿
    def get_robot_pose2d(self):
        return self.pose.to_pose2d()
            
    # # 获取机械臂状态
    # def get_arm_status(self,arm_id:utilis.Device_id):
    #     if arm_id == utilis.Device_id.LEFT:
    #         return self.left_manipulation_status.arm_info.status
    #     elif arm_id == utilis.Device_id.RIGHT:
    #         return self.right_manipulation_status.arm_info.status
    
    # # 获取机械臂姿态
    # def get_arm_pose(self,arm_id:utilis.Device_id):
    #     if arm_id == utilis.Device_id.LEFT:
    #         return self.left_manipulation_status.arm_info.arm_pose
    #     elif arm_id == utilis.Device_id.RIGHT:
    #         return self.right_manipulation_status.arm_info.arm_pose
    

    # 获取arm姿态和id  
    def get_msg_arms_pose_with_id(self,arm_id:utilis.Device_id):
        arm_poses = [msg.ArmPose]
        # 左臂
        if arm_id == utilis.Device_id.LEFT:
            arm_poses.append(self.left_manipulation_status.arm_info.arm_pose.to_msg_with_id())
            arm_poses.append(self.right_manipulation_status.arm_info.arm_pose.to_msg_with_id())
        # 右臂
        elif arm_id == utilis.Device_id.RIGHT:
            arm_poses.append(self.right_manipulation_status.arm_info.arm_pose.to_msg_with_id())
        # 左右臂
        elif arm_id == utilis.Device_id.LEFT_RIGHT:
            arm_poses.append(self.left_manipulation_status.arm_info.arm_pose.to_msg_with_id())
            arm_poses.append(self.right_manipulation_status.arm_info.arm_pose.to_msg_with_id())
        return arm_poses
    
    # 获取机器人状态
    def get_robot_status(self):
        return self.robot_status
    
    # 判断手臂是否空闲
    def is_arm_idle(self,arm_id:utilis.Device_id):
        if arm_id == utilis.Device_id.LEFT:
            return self.left_manipulation_status.arm_info.arm_status == manipulation_status.arm.status.IDLE
        elif arm_id == utilis.Device_id.RIGHT:
            return self.right_manipulation_status.arm_info.arm_status == manipulation_status.arm.status.IDLE
        elif arm_id == utilis.Device_id.LEFT_RIGHT:
            return self.left_manipulation_status.arm_info.arm_status == manipulation_status.arm.status.IDLE and \
                self.right_manipulation_status.arm_info.arm_status == manipulation_status.arm.status.IDLE
    # 更新机械臂状态
    def update_arm_status(self,arm_id:utilis.Device_id,arm_status:manipulation_status.arm.status):
        if arm_id == utilis.Device_id.LEFT:
            self.left_manipulation_status.arm_info.arm_status     = arm_status
        elif arm_id == utilis.Device_id.RIGHT:
            self.right_manipulation_status.arm_info.arm_status    = arm_status
        elif arm_id == utilis.Device_id.LEFT_RIGHT:
            self.left_manipulation_status.arm_info.arm_status     = arm_status
            self.right_manipulation_status.arm_info.arm_status    = arm_status
    
    # 更新机器人状态
    def update_robot_status(self,robot_status:Robot_status):
        self.robot_status = robot_status    