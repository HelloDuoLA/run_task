#!/usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import annotations
# import rospy
from enum import Enum,auto
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
# from geometry_msgs.msg import Pose
import geometry_msgs
import run_task.msg as msg
# import run_task.srv as srv

# 2D位姿数据结构
class Pose2D():
    def __init__(self,x=-999,y=-999,theta=-999):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f"x:{self.x} y:{self.y} theta:{self.theta}"
    
    def set_x(self,x):
        self.x = x
    
    def set_y(self,y):
        self.y = y
    
    def set_theta(self,theta):
        self.theta = theta
        
    def to_pose3d(self):
        return Pose3D(self.x, self.y, 0, 0, 0, self.theta)
        

# 3D位姿数据结构
class Pose3D():
    def __init__(self,x=-999,y=-999,z=-999,roll=-999,pitch=-999,yaw=-999):
        self.x     = x
        self.y     = y
        self.z     = z
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw

    def __str__(self):
        return f"x:{self.x:.2f} y:{self.y:.2f} z:{self.z:.2f} roll:{self.roll:.2f} pitch:{self.pitch:.2f} yaw:{self.yaw:.2f}"
    
    # 通过四元数初始化
    @staticmethod
    def instantiate_by_geometry_msg(pose:geometry_msgs.msg.Pose):
        pose3D = Pose3D()
        pose3D.roll, pose3D.pitch, pose3D.yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
        pose3D.x = pose.position.x
        pose3D.y = pose.position.y
        pose3D.z = pose.position.z
        return pose3D
    
    # 通过2D位姿初始化
    @staticmethod
    def instantiate_by_pose2d(pose2d:Pose2D):
        pose3D = Pose3D()
        pose3D.x = pose2d.x
        pose3D.y = pose2d.y
        pose3D.z = 0
        pose3D.yaw   = pose2d.theta
        pose3D.roll  = 0
        pose3D.pitch = 0
    
    @staticmethod
    def instantiate_by_xyz_orientation(x,y,z,o_x,o_y,o_z,o_w):
        pose3D   = Pose3D()
        pose3D.x = x
        pose3D.y = y
        pose3D.z = z
        pose3D.roll, pose3D.pitch, pose3D.yaw = euler_from_quaternion((o_x, o_y, o_z, o_w))
        return pose3D
    
    # 数据转换为geometry_msg
    def to_geometry_msg(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose
    
    # 输出为2D位姿
    def to_pose2d(self):
        return Pose2D(self.x, self.y, self.yaw)

# 设备id类
class Device_id(Enum):
    TBD        = 0        # 待定
    LEFT       = auto()   # 左
    RIGHT      = auto()   # 右 
    LEFT_RIGHT = auto()   # 左右
    LEFT_OR_RIGHT = auto()# 左或右
    HEAD       = auto()   # 头(头部摄像头)
    BOTTOM     = auto()   # 底部(底部摄像头)
    
    
    def __str__(self) -> str:
        return self.name
    
    def __eq__(self, value: Device_id) -> bool:
        if isinstance(value, Device_id):
            return self.value == value.value
        elif isinstance(value, int):
            return self.value == value


# 订阅的话题名称
class Topic_name():
    image_recognition_request = "/image_rec_request"     # 图像识别请求
    image_recognition_result  = "/image_rec_result"      # 图像识别结果
    camera_status             = "/camera_status"         # 摄像头状态
    left_arm_pose             = "/left_arm_pose"         # 左机械臂状态
    right_arm_pose            = "/right_arm_pose"        # 右机械臂状态
    clamp_status              = "/clamp_status"          # 夹爪状态
    image_model_status        = "/image_model_status"    # 图像模型状态
    voice_model_status        = "/voice_model_status"    # 语音模型状态
    robot_status              = "/robot_status"          # 机器人状态
    make_order                = "/make_order"            # 下单
    left_arm_action           = "/left_arm_action"       # 左臂action
    right_arm_action          = "/right_arm_action"      # 右臂action
    left_camera_raw_image     = "/left_camera_raw_image" # 左摄像头原始图像
    right_camera_raw_image    = "/right_camera_raw_image"# 右摄像头原始图像
    head_camera_raw_image     = "/head_camera_raw_image" # 头摄像头原始图像
    bottom_camera_raw_image   = "/bottom_camera_raw_image"# 底部摄像头原始图像
    check_arm_pose            = "/check_arm_pose"        # 检查机械臂状态
