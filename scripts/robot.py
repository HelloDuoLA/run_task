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
    # TODO:缺少机械臂初始位姿 
    # 初始化
    def __init__(self,id:utilis.Device_id):
        self.id          = id                # 机械臂编号
        self.arm_info    = self.arm()        # 机械臂状态
        # self.clamp_info  = self.clamp()      # 夹具信息
        self.camera_info = self.camera()     # 摄像头信息
        
    # 机械臂状态
    class arm():
        class status(Enum):
            IDLE   = 0            # 空闲
            MOVING = auto()       # 正在移动
            BUSY   = auto()       # 忙碌
            
        def __init__(self,arm_pose:arm.Arm_pose=arm.Arm_pose()):
            self.status   = self.status.IDLE    # 机械臂状态,默认空闲
            self.arm_pose = arm_pose            # 机械臂位置
            
        def set_status(self,status):
            self.status = status
        
        def set_moving(self):
            self.status = self.status.Moving
            
        def set_idle(self):
            self.status = self.status.IDLE

    # # 夹具类
    # class clamp():
    #     class status(Enum):
    #         OPEN    = 0           # 已经打开
    #         CLOSE   = auto()      # 已经关闭
    #         OPENING = auto()      # 正在打开
    #         CLOSING = auto()      # 正在关闭
    #         DONTCANGE = auto()    # 不改变
            
    #         def __str__(self) -> str:
    #             return self.name
            
    #         def __eq__(self, value: object) -> bool:
    #             if isinstance(value, self.__class__):
    #                 return self.value == value.value
    #             elif isinstance(value, int):
    #                 return self.value == value
            
        
    #     def __init__(self):
    #         # TODO:可能实际情况默认不是打开
    #         self.status = self.status.OPEN  # 夹具状态,默认打开
            
    #     def set_status(self,status):
    #         self.status = status
    
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
        
    class Common_status(Enum):
        IDLE      = 0           # 空闲
        BUSY      = auto()      # 忙碌
        DONCHANGE = auto()      # 不改变
    
    def __init__(self,pose:utilis.Pose3D=utilis.Pose3D()) -> None:
        self.robot_status              = self.Robot_status.IDLE                         # 机器人状态
        self.right_manipulation_status = manipulation_status(utilis.Device_id.RIGHT)    # 右臂
        self.left_manipulation_status  = manipulation_status(utilis.Device_id.LEFT)     # 左臂
        # self.pose = pose                                                                # 3D位姿
        self.image_recognition_status  = self.Common_status.IDLE                        # 图像识别状态,默认空闲
        self.voice_recognition_status  = self.Common_status.IDLE                        # 语音识别状态,默认空闲
        
        # 状态信息订阅
        # TODO:细节还需处理, 话题名称、消息类型
        # 摄像头状态更新
        rospy.Subscriber(utilis.Topic_name.camera_status , msg.CommonStatusDouble, self.update_camera_status, queue_size=10)
        # 机械臂状态更新
        rospy.Subscriber(utilis.Topic_name.right_arm_pose, msg.ArmPoseWithID, self.update_arm_status,    queue_size=10)
        rospy.Subscriber(utilis.Topic_name.left_arm_pose , msg.ArmPoseWithID, self.update_arm_status,    queue_size=10)
        # 爪具状态更新
        # rospy.Subscriber(utilis.Topic_name.clamp_status  , msg.CommonStatusDouble, self.update_clamp_status,  queue_size=10)
        # 图像模型状态更新 
        rospy.Subscriber(utilis.Topic_name.image_model_status, msg.CommonStatus, self.update_image_model_status_status,  queue_size=10)
        # 语音模型状态更新 
        rospy.Subscriber(utilis.Topic_name.image_model_status, msg.CommonStatus, self.update_image_model_status_status,  queue_size=10)
        # TODO:可能需要通过TF解决
        # 机器人位姿更新
        # rospy.Subscriber(utilis.Topic_name.robot_status, msg.Pose3D, self.update_robot_pose3d, queue_size=10)

    # 获取2D位姿
    def get_robot_pose2d(self):
        return self.pose.to_pose2d()
            
    # 获取机械臂状态
    def get_arm_status(self,arm_id:utilis.Device_id):
        if arm_id == utilis.Device_id.LEFT:
            return self.left_manipulation_status.arm_info.status
        elif arm_id == utilis.Device_id.RIGHT:
            return self.right_manipulation_status.arm_info.status
    
    # 获取机械臂姿态
    def get_arm_pose(self,arm_id:utilis.Device_id):
        if arm_id == utilis.Device_id.LEFT:
            return self.left_manipulation_status.arm_info.arm_pose
        elif arm_id == utilis.Device_id.RIGHT:
            return self.right_manipulation_status.arm_info.arm_pose
    
    # 获取机械臂信息(姿态和状态)
    def get_arm_info(self,arm_id:utilis.Device_id):
        if arm_id == utilis.Device_id.LEFT:
            return self.left_manipulation_status.arm_info
        elif arm_id == utilis.Device_id.RIGHT:
            return self.right_manipulation_status.arm_info
    
    # 获取arm状态
    def get_arm_status(self,arm_id:utilis.Device_id):
        if arm_id == utilis.Device_id.LEFT:
            return self.left_manipulation_status.arm_info.status
        elif arm_id == utilis.Device_id.RIGHT:
            return self.right_manipulation_status.arm_info.status
    
    # 获取arm 姿态
    def get_arm_pose(self,arm_id:utilis.Device_id):
        if arm_id == utilis.Device_id.LEFT:
            return self.left_manipulation_status.arm_info.arm_pose
        elif arm_id == utilis.Device_id.RIGHT:
            return self.right_manipulation_status.arm_info.arm_pose
    
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
    
    # TODO:更新状态均需要通过话题订阅!!!!!
    # 更新3D位姿
    def update_robot_pose3d(self,pose:utilis.Pose3D):
        self.pose = pose
    
    # 更新机械臂状态
    def update_arm_status(self,arm_id:utilis.Device_id,arm_status:manipulation_status.arm.status):
        if arm_id == utilis.Device_id.LEFT:
            self.left_manipulation_status.arm_info.status     = arm_status
        elif arm_id == utilis.Device_id.RIGHT:
            self.right_manipulation_status.arm_info.status    = arm_status
        elif arm_id == utilis.Device_id.LEFT_RIGHT:
            self.left_manipulation_status.arm_info.status     = arm_status
            self.right_manipulation_status.arm_info.status    = arm_status
        
    def update_robot_status(self,robot_status:Robot_status):
        self.robot_status = robot_status
    
    # # 更新夹具状态
    # def update_clamp_status(self,arm_id:utilis.Device_id,clamp_status:manipulation_status.clamp.status):
    #     if arm_id == utilis.Device_id.LEFT:
    #         self.left_manipulation_status.clamp_info.status = clamp_status
    #     elif arm_id == utilis.Device_id.RIGHT:
    #         self.right_manipulation_status.clamp_info.status = clamp_status
    
    # 更新摄像头状态
    def update_camera_status(self,arm_id:utilis.Device_id,camera_status:manipulation_status.camera.status):
        if arm_id == utilis.Device_id.LEFT:
            self.left_manipulation_status.camera_info.status = camera_status
        elif arm_id == utilis.Device_id.RIGHT:
            self.right_manipulation_status.camera_info.status = camera_status

    # 更新机械臂和夹具状态
    # def update_arm_clamp_status(self,arm_id:utilis.Device_id,arm_status:manipulation_status.arm.status,arm_pose:arm.Arm_pose,clamp_status:manipulation_status.clamp.status):
    #     self.update_arm_status(arm_id,arm_status,arm_pose)
        # self.update_clamp_status(arm_id,clamp_status)
    
    # 更新图像识别模型状态
    def update_image_model_status_status():
        pass
    
    # 更新语言识别模型状态
    def update_voice_model_status_status():
        pass
    
    