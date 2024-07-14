#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import actionlib
import random
from pymycobot import Mercury
from enum import Enum,auto # 任务字典
import time
import copy

# 增加头文件路径 
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import run_task.msg as msg
import run_task.srv as srv

# 手臂移动方式
class ArmMoveMethod(Enum):
    XYZ        = 0
    X_YZ       = auto()
    X_Y_Z      = auto()
    X_Z_Y      = auto()
    Y_XZ       = auto()
    Y_X_Z      = auto()
    Y_Z_X      = auto()
    Z_XY       = auto()
    Z_X_Y      = auto()
    Z_Y_X      = auto()
    XY_Z       = auto()
    XZ_Y       = auto()
    YZ_X       = auto()
    # ONLY_X     = auto()
    # ONLY_Y     = auto()
    # ONLY_Z     = auto()
    MODIFY_X   = auto()
    MODIFY_Y   = auto()
    MODIFY_Z   = auto()
    OPLY_GRIP  = auto()
    X_Y_Z_OTHER= auto()
    Z_X_Y_OTHER= auto()
    
    
    def __str__(self) -> str:
        return self.name
    
    def __eq__(self, value: object) -> bool:
        if isinstance(value, self.__class__):
            return self.value == value.value
        elif isinstance(value, int):
            return self.value == value

# 抓取方式
class GripMethod(Enum):
    DONTCANGE  = 0       # 不改变
    OPEN       = auto()  # 最后打开
    OPEN_FIRST = auto()  # 先打开
    CLOSE      = auto()  # 最后关闭
    CLOSE_FIRST= auto()  # 先关闭
    OPEN_CLOSE = auto()  # 先打开后关闭
    CLOSE_OPEN = auto()  # 先关闭后打开
    
    def __str__(self) -> str:
        return self.name
    
    def __eq__(self, value: object) -> bool:
        if isinstance(value, self.__class__):
            return self.value == value.value
        elif isinstance(value, int):
            return self.value == value



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
            return f"arm {self.arm_id}, pose type {self.type_id}, joint:{self.arm_pose}"
        elif self.type_id == PoseType.BASE_COORDS:
            return f"arm {self.arm_id}, pose type {self.type_id}, coords:{self.arm_pose}"
    # 将列表状态输出为action的数据结构
    def list_to_msg(self):
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
            self.request_name     = utilis.Topic_name.left_arm_topic             # 左臂移动请求话题名称
            self.result_name      = utilis.Topic_name.left_arm_result            # 左臂移动结果话题名称
            self.control_instance = Mercury("/dev/left_arm") 
            self.arm_name = "left_arm"             
        elif(self.id == utilis.Device_id.RIGHT):
            self.action_name      = utilis.Topic_name.right_arm_action           # action名称
            self.request_name     = utilis.Topic_name.right_arm_topic            # 右臂移动请求话题名称
            self.result_name      = utilis.Topic_name.right_arm_result           # 右臂移动结果话题名称
            self.control_instance = Mercury("/dev/right_arm")                    
            self.arm_name = "right_arm"     
        
        power_on = self.is_power_on()
        
        rospy.loginfo(f"{self.arm_name} is power status {power_on}")
        while power_on == False:
            rospy.loginfo(f"{self.arm_name} is power status {power_on}")
            self.control_instance.power_off()
            time.sleep(0.3)
            self.control_instance.power_on()
            power_on = self.is_power_on()
            
            
        # 开启机械抓爪通信
        self.control_instance.set_gripper_mode(0)

        
        # self.action = self.arm_action(self.action_name,self.control_instance,id)
        # self.action.start_action()
        self.arm_topics = self.arm_topic(self.request_name,self.result_name,self.control_instance,id)
        
    
    def is_power_on(self):
        # 检查数组长度是否为13
        status = self.control_instance.get_robot_status()
        if not isinstance(status, list):
            return False
        # 然后检查列表中的每个元素是否都为0
        return all(x == 0 for x in status)
    
    # 机械臂 action 
    # class arm_action():
    class arm_topic():
        # def __init__(self,action_name,control_instance, id) -> None:
        def __init__(self,request_name,result_name,control_instance, id) -> None:
            self.id               = id
            # self.action_server    = actionlib.SimpleActionServer(action_name, msg.MoveArmAction, self.execute_cb, False)
            self.control_instance = control_instance
            # rospy.loginfo(f"node: {rospy.get_name()}, init {action_name} arm action server")
            self.pub = rospy.Publisher(result_name, msg.ArmMoveResult,queue_size=10)         # 发布移动结果
            self.sub = rospy.Subscriber(request_name,msg.ArmMoveRequest,self.execute_cb,callback_args=self,queue_size=10)        # 订阅移动请求 
        
        # 启动action
        def start_action(self):    
            self.action_server.start()
        
        # 执行
        @staticmethod
        # def execute_cb(self, goal:msg.MoveArmGoal):
        def execute_cb(goal:msg.ArmMoveRequest,self):
            # rospy.loginfo(f"node: {rospy.get_name()}, arm action server execute. goal: {goal}")
            rospy.loginfo(f"node: {rospy.get_name()}, {self.id} arm execute.task id {goal.task_index}")
            # 1. 解析目标值
            goal_arm_pose     = goal.arm_pose
            goal_grasp_flag   = goal.grasp_flag
            
            # 先打开 and 先打开后关闭
            if goal_grasp_flag == GripMethod.OPEN_CLOSE or goal_grasp_flag == GripMethod.OPEN_FIRST:
                self.open_grasp()
            # 先关闭 and 先关闭后打开
            elif goal_grasp_flag == GripMethod.CLOSE_OPEN or goal_grasp_flag == GripMethod.CLOSE_FIRST:
                self.close_grasp()
            
            
            if goal.arm_move_method != ArmMoveMethod.OPLY_GRIP:
                # 2. 给机械臂发送目标值
                self.move_arm(goal_arm_pose.type_id,goal_arm_pose.arm_pose, goal.arm_move_method)
            
            # 后打开 and 先关闭后打开
            if goal_grasp_flag == GripMethod.CLOSE_OPEN or goal_grasp_flag == GripMethod.OPEN:
                self.open_grasp()
            # 后关闭 and 先打开后关闭
            elif goal_grasp_flag == GripMethod.OPEN_CLOSE or goal_grasp_flag == GripMethod.CLOSE:
                self.close_grasp()
        
            
            # 构建返回数据
            # result = msg.MoveArmResult()
            result = msg.ArmMoveResult()
            result.arm_id     = goal.arm_id
            result.task_index = goal.task_index
            rospy.loginfo(f"result : {result}")
            try:
                # self.action_server.set_succeeded(result) #可以添加结果参数
                self.pub.publish(result)
            except Exception as e:
                print(f"Exception in done_cb: {e}")
                
        
        # 关闭抓爪 
        def close_grasp(self):
            result = self.control_instance.set_gripper_state(1,100)
            time.sleep(1)
            return result
        
        # 打开抓爪 
        def open_grasp(self):
            result = self.control_instance.set_gripper_state(0,100)
            time.sleep(1)
            return result
            
        # 机械臂移动 
        def move_arm(self,pose_type:PoseType,target_pose,move_method:ArmMoveMethod):
            rospy.loginfo(f"{self.id} arm move to {target_pose} using {pose_type} method {move_method}")
            arm_speed =  50
            if pose_type == PoseType.ANGLE:
                result = self.control_instance.send_angles(target_pose,arm_speed)
                self.wait()
                return result 
            elif pose_type == PoseType.BASE_COORDS:
                if move_method == ArmMoveMethod.XYZ:
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                elif move_method == ArmMoveMethod.X_YZ:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_x = copy.deepcopy(current_base_coords)
                    base_coords_change_x[0] = target_pose[0]
                    result = self.control_instance.send_base_coords(base_coords_change_x,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                    
                elif move_method == ArmMoveMethod.X_Y_Z:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_x = copy.deepcopy(current_base_coords)
                    base_coords_change_x[0] = target_pose[0]
                    result = self.control_instance.send_base_coords(base_coords_change_x,arm_speed)
                    self.wait()
                    
                    base_coords_change_y = copy.deepcopy(base_coords_change_x)
                    base_coords_change_y[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_y,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                    
                elif move_method == ArmMoveMethod.X_Z_Y:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_x = copy.deepcopy(current_base_coords)
                    base_coords_change_x[0] = target_pose[0]
                    result = self.control_instance.send_base_coords(base_coords_change_x,arm_speed)
                    self.wait()
                    
                    base_coords_change_z = copy.deepcopy(base_coords_change_x)
                    base_coords_change_z[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                    
                elif move_method == ArmMoveMethod.Y_XZ:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_y = copy.deepcopy(current_base_coords)
                    base_coords_change_y[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_y,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                    
                elif move_method == ArmMoveMethod.Y_X_Z:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_y = copy.deepcopy(current_base_coords)
                    base_coords_change_y[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_y,arm_speed)
                    self.wait()
                    
                    base_coords_change_x = copy.deepcopy(base_coords_change_y)
                    base_coords_change_x[0] = target_pose[0]
                    result = self.control_instance.send_base_coords(base_coords_change_x,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                    
                elif move_method == ArmMoveMethod.Y_Z_X:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_y = copy.deepcopy(current_base_coords)
                    base_coords_change_y[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_y,arm_speed)
                    self.wait()
                    
                    base_coords_change_z = copy.deepcopy(base_coords_change_y)
                    base_coords_change_z[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                elif move_method == ArmMoveMethod.Z_XY:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_z = copy.deepcopy(current_base_coords)
                    base_coords_change_z[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                elif move_method == ArmMoveMethod.Z_X_Y:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_z = copy.deepcopy(current_base_coords)
                    base_coords_change_z[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    base_coords_change_x = copy.deepcopy(base_coords_change_z)
                    base_coords_change_x[0] = target_pose[0]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                elif move_method == ArmMoveMethod.Z_Y_X:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_z = copy.deepcopy(current_base_coords)
                    base_coords_change_z[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    base_coords_change_y = copy.deepcopy(base_coords_change_z)
                    base_coords_change_y[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                elif move_method == ArmMoveMethod.XY_Z:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_xy = copy.deepcopy(current_base_coords)
                    base_coords_change_xy[0] = target_pose[0]
                    base_coords_change_xy[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_xy,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                    
                elif move_method == ArmMoveMethod.XZ_Y:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_xz = copy.deepcopy(current_base_coords)
                    base_coords_change_xz[0] = target_pose[0]
                    base_coords_change_xz[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_xz,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                elif move_method == ArmMoveMethod.YZ_X:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_yz = copy.deepcopy(current_base_coords)
                    base_coords_change_yz[1] = target_pose[1]
                    base_coords_change_yz[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_yz,arm_speed)
                    self.wait()
                    
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()  
                # elif move_method == ArmMoveMethod.ONLY_Z:
                #     current_base_coords = self.get_base_coords()
                #     current_base_coords[2] = target_pose[2]
                #     result = self.control_instance.send_base_coords(current_base_coords,arm_speed)
                #     self.wait()
                elif move_method == ArmMoveMethod.MODIFY_Z:
                    current_base_coords = self.get_base_coords()
                    current_base_coords[2] += target_pose[2]
                    result = self.control_instance.send_base_coords(current_base_coords,arm_speed)
                    self.wait()
                # 先移动x轴, 再然后动Y,Z,其他全部，包括角度旋转
                elif move_method == ArmMoveMethod.X_Y_Z_OTHER:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_x = copy.deepcopy(current_base_coords)
                    base_coords_change_x[0] = target_pose[0]
                    result = self.control_instance.send_base_coords(base_coords_change_x,arm_speed)
                    self.wait()
                    
                    base_coords_change_y = copy.deepcopy(base_coords_change_x)
                    base_coords_change_y[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_y,arm_speed)
                    self.wait()
                    
                    base_coords_change_z = copy.deepcopy(base_coords_change_y)
                    base_coords_change_z[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    # change other
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                # 先移动Z轴, 再然后动其他全部，包括角度旋转
                elif move_method == ArmMoveMethod.Z_X_Y_OTHER:
                    current_base_coords = self.get_base_coords()
                    base_coords_change_z = copy.deepcopy(current_base_coords)
                    base_coords_change_z[2] = target_pose[2]
                    result = self.control_instance.send_base_coords(base_coords_change_z,arm_speed)
                    self.wait()
                    
                    base_coords_change_x = copy.deepcopy(base_coords_change_z)
                    base_coords_change_x[0] = target_pose[0]
                    result = self.control_instance.send_base_coords(base_coords_change_x,arm_speed)
                    self.wait()
                    
                    base_coords_change_y = copy.deepcopy(base_coords_change_z)
                    base_coords_change_y[1] = target_pose[1]
                    result = self.control_instance.send_base_coords(base_coords_change_y,arm_speed)
                    self.wait()
                    
                    # change other
                    result = self.control_instance.send_base_coords(target_pose,arm_speed)
                    self.wait()
                    
                else:
                    raise ValueError("Invalid move method")
        
                return result 
        # 等待机械臂运动结束
        def wait(self):
            time.sleep(0.3)
            while(self.control_instance.is_moving()):
                # print("arm is moving")
                time.sleep(0.03)
                
        #  可靠地获取当前的基座标
        def get_base_coords(self):
            current_base_coords     = self.control_instance.get_base_coords()
            while current_base_coords == None:
                current_base_coords = self.control_instance.get_base_coords()
                time.sleep(0.3)
            return current_base_coords
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
    rate = rospy.Rate(0.1)

    
    while not rospy.is_shutdown():
        rospy.loginfo(f"{arm_name}_arm_node")
        rate.sleep()

# 服务函数 
def doCheckArmPose(req:srv.CheckArmPoseRequest):
    rospy.loginfo(f"node: {rospy.get_name()}, doCheckArmPose. req: {req}")
    resp = srv.CheckArmPoseResponse()
    resp.type_id = req.type_id
    if req.type_id == PoseType.ANGLE:
        result = arm_controller.control_instance.get_angles()
        while result == None:
            result = arm_controller.control_instance.get_angles()
            time.sleep(0.3)
        resp.arm_pose = result
    elif req.type_id == PoseType.BASE_COORDS:
        result = arm_controller.control_instance.get_base_coords()
        while result == None:
            result = arm_controller.control_instance.get_base_coords()
            time.sleep(0.3)
        resp.arm_pose = result

    rospy.loginfo(f"node: {rospy.get_name()}, doCheckArmPose. req: {req} resp arm : {resp.arm_pose}")
    return resp

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
