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
        return f"x:{self.x} y:{self.y} z:{self.z} roll:{self.roll} pitch:{self.pitch} yaw:{self.yaw}"
    
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
        return self.value == value.value


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

# 机械臂位姿
class Arm_pose():
    # 初始化
    def __init__(self,arm_pose:msg.ArmPoseWithID=0):
        if isinstance(arm_pose, msg.ArmPose):
            self.x   = arm_pose.x
            self.y   = arm_pose.y
            self.z   = arm_pose.z
            self.rx  = arm_pose.rx
            self.ry  = arm_pose.ry
            self.rz  = arm_pose.rz
            self.arm_id = Device_id.TBD
        elif isinstance(arm_pose,msg.ArmPoseWithID):
            self.x   = arm_pose.x
            self.y   = arm_pose.y
            self.z   = arm_pose.z
            self.rx  = arm_pose.rx
            self.ry  = arm_pose.ry
            self.rz  = arm_pose.rz
            self.arm_id = arm_pose.id
        elif isinstance(arm_pose, list) and len(arm_pose) == 6:
            self.x, self.y, self.z, self.rx, self.ry, self.rz = arm_pose
            self.arm_id = Device_id.TBD
        elif arm_pose == 0:
            self.x  = -999
            self.y  = -999
            self.z  = -999
            self.rx = -999
            self.ry = -999
            self.rz = -999 
            self.arm_id = Device_id.TBD
        else:
            raise ValueError("Invalid initialization parameter for arm_pose")
    
    def set_x(self,x):
        self.x = x
    
    def set_y(self,y):
        self.y = y
        
    def set_z(self,z):
        self.z = z
    
    def set_rx(self,rx):
        self.rx = rx
    
    def set_ry(self,ry):
        self.ry = ry
    
    def set_rz(self,rz):
        self.rz = rz
        
    def set_id(self,arm_id:Device_id):
        self.arm_id = arm_id

    # 重写等号
    def __eq__(self, other):
        return (self.x == other.x and
                self.y == other.y and
                self.z == other.z and
                self.rx == other.rx and
                self.ry == other.ry)
    
    # 重写打印输出
    def __str__(self):
        return f"x:{self.x} y:{self.y} z:{self.z} rx:{self.rx} ry:{self.ry} rz:{self.rz}"

    @staticmethod
    # 将列表状态输出为action的数据结构
    def list_to_msg(arm_list_status:list,arm_id:Device_id):
            arm_pose = msg.ArmPose()
            arm_pose.x   = arm_list_status[0]
            arm_pose.y   = arm_list_status[1]
            arm_pose.z   = arm_list_status[2]
            arm_pose.rx  = arm_list_status[3]
            arm_pose.ry  = arm_list_status[4]
            arm_pose.rz  = arm_list_status[5]
            return arm_pose
    
    # 转为msg.ArmPose
    def to_msg(self):
        arm_pose = msg.ArmPose()
        arm_pose.x   = self.x
        arm_pose.y   = self.y
        arm_pose.z   = self.z
        arm_pose.rx  = self.rx
        arm_pose.ry  = self.ry
        arm_pose.rz  = self.rz
        return arm_pose

    # 转为msg.ArmPoseWithID
    def to_msg_with_id(self,id:Device_id=None):
        arm_pose = msg.ArmPoseWithID()
        arm_pose.x   = self.x
        arm_pose.y   = self.y
        arm_pose.z   = self.z
        arm_pose.rx  = self.rx
        arm_pose.ry  = self.ry
        arm_pose.rz  = self.rz
        if isinstance(id,Device_id):
            self.arm_id = id
        arm_pose.id  = self.arm_id
        return arm_pose
    
    def to_list(self):
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]
    
    
class Arm_pose_angle():
    def __init__(self,arm_angle:list=0):
        if arm_angle == 0:
            pass
        else:
            self.angle1 = arm_angle[0]
            self.angle2 = arm_angle[1]
            self.angle3 = arm_angle[2]
            self.angle4 = arm_angle[3]
            self.angle5 = arm_angle[4]
            self.angle6 = arm_angle[5]
    
    def set_angle1(self,angle1):
        self.angle1 = angle1
    
    def set_angle2(self,angle2):
        self.angle2 = angle2
        
    def set_angle3(self,angle3):
        self.angle3 = angle3
        
    def set_angle4(self,angle4):
        self.angle4 = angle4
        
    def set_angle5(self,angle5):
        self.angle5 = angle5
        
    def set_angle6(self,angle6):
        self.angle6 = angle6
    
    def to_list(self):
        return [self.angle1,self.angle2,self.angle3,self.angle4,self.angle5,self.angle6]
    
    def to_msg_with_id(self,id:Device_id=None):
        arm_pose_msg = msg.ArmPoseWithID()
        arm_pose_msg.x    = self.angle1
        arm_pose_msg.y    = self.angle2
        arm_pose_msg.z    = self.angle3
        arm_pose_msg.rx   = self.angle4
        arm_pose_msg.ry   = self.angle5
        arm_pose_msg.rz   = self.angle6
        arm_pose_msg.type = 1 # 1表示角度
        if isinstance(id,Device_id):
            self.arm_id = id
        arm_pose_msg.id  = self.arm_id
        return arm_pose_msg