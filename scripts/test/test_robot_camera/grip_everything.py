#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import actionlib
import copy

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
import arm



# 抓取一切
class Grip_everything():
    def __init__(self) -> None:
        self._initialize_arm_anchor_point()
    
    def _initialize_arm_anchor_point(self):
        self.left_arm_idle                 = self._get_arm_anchor_angle("LeftArmIdle")                # 左臂闲置
        self.left_arm_container_rec        = self._get_arm_anchor_angle("LeftArmContainerRec")        # 左臂识别容器
        self.left_arm_snack_rec            = self._get_arm_anchor_angle("LeftArmSnackRec")            # 左臂零食识别
        self.left_arm_snack_grip           = self._get_arm_anchor_coord("LeftArmGripSnack")           # 左臂零食抓取
        self.left_arm_snack_placement      = self._get_arm_anchor_coord("LeftArmSnackPlacement")      # 左臂零食放置
        self.left_arm_container_grip_pre   = self._get_arm_anchor_coord("LeftArmGripContainerPre")    # 左臂抓取容器准备状态
        self.left_arm_container_grip       = self._get_arm_anchor_coord("LeftArmGripContainer")       # 左臂抓取容器
        self.left_arm_container_delivery   = self._get_arm_anchor_coord("LeftArmContainerDelivery")   # 左臂容器运送时的姿态
        self.left_arm_container_placement  = self._get_arm_anchor_coord("LeftArmContainerPlacement")  # 左臂容器放置
        self.left_arm_machine_turn_on_rec  = self._get_arm_anchor_angle("LeftArmMachineTurnOnRec")    # 左臂 开 咖啡机识别
        self.left_arm_machine_turn_off_rec = self._get_arm_anchor_angle("LeftArmMachineTurnOFFRec")   # 左臂 关 咖啡机识别
        self.left_arm_machine_turn_on_pre  = self._get_arm_anchor_coord("LeftArmMachineTurnOnPre")    # 左臂 开 咖啡机预备动作
        self.left_arm_machine_turn_off_pre = self._get_arm_anchor_coord("LeftArmMachineTurnOffPre")   # 左臂 关 咖啡机预备动作
        
        
        self.right_arm_idle                = self._get_arm_anchor_angle("RightArmIdle")               # 右臂空闲
        self.right_arm_container_rec       = self._get_arm_anchor_angle("RightArmContainerRec")       # 右臂识别容器
        self.right_arm_snack_rec           = self._get_arm_anchor_angle("RightArmSnackRec")           # 右臂识别零食
        self.right_arm_snack_grip          = self._get_arm_anchor_coord("RightArmGripSnack")          # 右臂零食抓取
        self.right_arm_snack_placement     = self._get_arm_anchor_coord("RightArmSnackPlacement")     # 右臂零食放置
        self.right_arm_container_grip_pre  = self._get_arm_anchor_coord("RightArmGripContainerPre")   # 右臂抓取容器准备状态
        self.right_arm_container_grip      = self._get_arm_anchor_coord("RightArmGripContainer")      # 右臂抓取容器
        self.right_arm_container_delivery  = self._get_arm_anchor_coord("RightArmContainerDelivery")  # 右臂容器运送时的姿态
        self.right_arm_container_placement = self._get_arm_anchor_coord("RightArmContainerPlacement") # 右臂容器放置
        self.right_arm_cup_rec_pre         = self._get_arm_anchor_angle("RightArmCupRecPre")          # 右臂杯子识别预备
        self.right_arm_cup_rec             = self._get_arm_anchor_angle("RightArmCupRec")             # 右臂杯子识别
        self.right_arm_cup_grab            = self._get_arm_anchor_coord("RightArmCupGrab")            # 右臂杯子夹取
        self.right_arm_cup_water           = self._get_arm_anchor_coord("RightArmCupWater")           # 右臂杯子接水
        self.right_arm_cup_delivery        = self._get_arm_anchor_coord("RightArmCupDelivery")        # 右臂杯子运送
        self.right_arm_cup_placement       = self._get_arm_anchor_coord("RightArmCupPlacement")       # 右臂杯子放置
        
    # 通过名字获取机械臂定位点
    def _get_arm_anchor_coord(self,anchor_point_name):
        if "left" in  anchor_point_name.lower() :
            arm_id = utilis.Device_id.LEFT
        elif "right" in  anchor_point_name.lower() :
            arm_id = utilis.Device_id.RIGHT
        else:
            arm_id = utilis.Device_id.TBD
        pose = arm.Arm_pose(rospy.get_param(f'~{anchor_point_name}/coords'),arm.PoseType.BASE_COORDS,arm_id)
        return pose
    
    # 通过名字获取机械臂定位点(角度)
    def _get_arm_anchor_angle(self,anchor_point_name):
        if "left" in  anchor_point_name.lower() :
            arm_id = utilis.Device_id.LEFT
        elif "right" in  anchor_point_name.lower() :
            arm_id = utilis.Device_id.RIGHT
        else:
            arm_id = utilis.Device_id.TBD
        pose = arm.Arm_pose(rospy.get_param(f'~{anchor_point_name}/angles'),arm.PoseType.ANGLE,arm_id)
        return pose
        

# 机械臂执行器 
class Manipulator_actuator():
    def __init__(self):
        self.left_arm_ac  = actionlib.SimpleActionClient(utilis.Topic_name.left_arm_action,  msg.MoveArmAction)
        self.right_arm_ac = actionlib.SimpleActionClient(utilis.Topic_name.right_arm_action, msg.MoveArmAction)
        rospy.loginfo("waiting for arm action server")
        self.left_arm_ac.wait_for_server()
        self.right_arm_ac.wait_for_server()
        self.running_tasks_manager = task.Task_manager_in_running() # 正在执行的任务管理器
    
    # 运行
    def run(self, manipulation_task:task.Task_manipulation):
        # 加在运行序列中
        task_index = self.running_tasks_manager.add_task(manipulation_task)
        rospy.loginfo(f"manipulation task {manipulation_task.task_index} is running ")
        # 任务开始
        manipulation_task.update_start_status()

        # 左臂
        if manipulation_task.arm_id == utilis.Device_id.LEFT:
            # 设置action 目标
            goal                      = msg.MoveArmGoal()
            goal.task_index           = task_index
            goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0].arm_pose
            goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            goal.grasp_speed          = manipulation_task.clamp_speed
            goal.arm_id               = manipulation_task.target_arms_pose[0].arm_id.value
            self.left_arm_ac.send_goal(goal,self.done_callback,self.active_callback,self.feedback_callback)
            rospy.loginfo(f"left arm goal:{goal}")
        # 右臂
        elif manipulation_task.arm_id == utilis.Device_id.RIGHT:
            # 设置action 目标
            goal                      = msg.MoveArmGoal()
            goal.task_index           = task_index
            goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0].arm_pose
            goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            goal.grasp_speed          = manipulation_task.clamp_speed
            goal.arm_id               = manipulation_task.target_arms_pose[0].arm_id.value
            self.right_arm_ac.send_goal(goal,self.done_callback,self.active_callback,self.feedback_callback)
            rospy.loginfo(f"right arm goal:{goal}")
        elif manipulation_task.arm_id == utilis.Device_id.LEFT_RIGHT:
            left_goal              = msg.MoveArmGoal()
            right_goal             = msg.MoveArmGoal()
            for i in range(2):
                if manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.LEFT:
                    left_goal.task_index           = task_index
                    left_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i].arm_pose
                    left_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    left_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    left_goal.grasp_speed          = manipulation_task.clamp_speed
                    left_goal.arm_id               = manipulation_task.target_arms_pose[i].arm_id.value
                elif manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.RIGHT:
                    right_goal.task_index           = task_index
                    right_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i].arm_pose
                    right_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    right_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    right_goal.grasp_speed          = manipulation_task.clamp_speed
                    right_goal.arm_id               = manipulation_task.target_arms_pose[i].arm_id.value
            rospy.loginfo(f"left arm goal:{left_goal}")
            rospy.loginfo(f"right arm goal:{right_goal}")
            self.left_arm_ac.send_goal(left_goal,self.done_callback,self.active_callback,self.feedback_callback)
            self.right_arm_ac.send_goal(right_goal,self.done_callback,self.active_callback,self.feedback_callback)


    # 完成回调
    @staticmethod
    def done_callback(status, result:msg.MoveArmResult):
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator done. status:{status} result:{result}")
    
    # 激活回调
    @staticmethod
    def active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator active")
    
    # 反馈回调
    @staticmethod
    def feedback_callback(feedback:msg.MoveArmFeedback):
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator feedback. {feedback}")
        

class Manipulator_tasks():
    def __init__(self) -> None:
        self.task_left_right_arms_idle = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE, None, utilis.Device_id.LEFT_RIGHT, \
            [grip_everything.left_arm_idle,grip_everything.right_arm_idle],\
            [arm.GripMethod.CLOSE,arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.XYZ)

        #  将左臂抬到指定位置(食物框识别位置)
        self.task_left_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.LEFT, \
            grip_everything.left_arm_container_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ)

        #  将右臂抬到指定位置(食物框识别位置)
        self.task_right_arm_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_container, None, utilis.Device_id.RIGHT,\
            grip_everything.right_arm_container_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ)

        #  将左臂抬到零食识别位置(可前后并行，固定)
        self.task_left_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.LEFT, \
            grip_everything.left_arm_snack_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ)
        
        #  将右臂抬到零食识别位置(可前后并行，固定)
        self.task_right_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_snack, None, utilis.Device_id.RIGHT, \
            grip_everything.right_arm_snack_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ)

        # 左臂夹取零食框
        self.task_left_arm_grap_container    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container,None,utilis.Device_id.LEFT,\
                [copy.deepcopy(grip_everything.left_arm_container_grip)],\
                    [arm.GripMethod.OPEN_CLOSE], arm_move_method = arm.ArmMoveMethod.XY_Z)

        # 右臂夹取零食框
        self.task_right_arm_grap_container    = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_container,None,utilis.Device_id.RIGHT,\
                [copy.deepcopy(grip_everything.right_arm_container_grip_pre)],\
                    [arm.GripMethod.OPEN_CLOSE], arm_move_method = arm.ArmMoveMethod.XY_Z)

        #  左、右臂将零食框放到指定高度
        self.task_arm_dilivery_container   = task.Task_manipulation(task.Task_type.Task_manipulation.Deliever_container,None,utilis.Device_id.LEFT_RIGHT,\
                [grip_everything.left_arm_container_delivery,grip_everything.right_arm_container_delivery], arm_move_method = arm.ArmMoveMethod.XYZ)
        
        #  将左、右臂放到指定位置后，松开
        self.task_arm_placement_container   = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_container,None,utilis.Device_id.LEFT_RIGHT,\
                [grip_everything.left_arm_container_placement,grip_everything.right_arm_container_placement],\
                [arm.GripMethod.OPEN,arm.GripMethod.OPEN], arm_move_method = arm.ArmMoveMethod.XYZ)

        #  将左,右臂放到空闲位置
        self.task_arms_idle   = task.Task_manipulation(task.Task_type.Task_manipulation.Move_to_IDLE,None,utilis.Device_id.LEFT_RIGHT,\
                [grip_everything.left_arm_idle,grip_everything.right_arm_idle],\
                [arm.GripMethod.CLOSE,arm.GripMethod.CLOSE], arm_move_method = arm.ArmMoveMethod.XYZ)

        #  左臂抬到指定位置识别咖啡机开关 开
        self.task_left_arm_to_rec_coffee_machine_turn_on = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_machine_switch, None, utilis.Device_id.LEFT, \
            grip_everything.left_arm_machine_turn_on_rec, arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ)

        #  右臂抬到指定位置  进行准备 ，识别杯子和机器
        self.task_right_arm_to_rec_cup_pre = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_cup_machine, None, utilis.Device_id.RIGHT,\
            grip_everything.right_arm_cup_rec_pre, arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ)

        #  右臂抬到指定位置识别杯子和机器
        self.task_right_arm_to_rec_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_cup_machine, None, utilis.Device_id.RIGHT,\
            grip_everything.right_arm_cup_rec, arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ)

        #  右臂夹取杯子(可并行，固定)
        self.task_right_arm_grasp_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Grasp_cup,None,utilis.Device_id.RIGHT,\
            grip_everything.right_arm_cup_grab,arm.GripMethod.OPEN_CLOSE, arm_move_method = arm.ArmMoveMethod.XYZ)

        # 左臂放置到按钮下方, 进行准备
        self.task_left_arm_turn_on_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_on_coffee_machine,None,utilis.Device_id.LEFT,\
            copy.deepcopy(grip_everything.left_arm_machine_turn_off_pre),arm.GripMethod.CLOSE,\
                arm_move_method = arm.ArmMoveMethod.Y_X_Z, click_length=10)

        #  右臂将杯子挪到咖啡机
        self.task_right_arm_water_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Water_cup,None,utilis.Device_id.RIGHT,\
            copy.deepcopy(grip_everything.right_arm_cup_water), arm_move_method = arm.ArmMoveMethod.X_YZ)

        #  左臂抬到指定位置识别咖啡机开关 关
        self.task_left_arm_to_rec_coffee_machine_turn_off = task.Task_manipulation(task.Task_type.Task_manipulation.Rec_machine_switch, None, utilis.Device_id.LEFT, \
            grip_everything.left_arm_machine_turn_on_rec, arm.GripMethod.CLOSE, arm_move_method = arm.ArmMoveMethod.X_YZ)

        # 左臂关闭咖啡机(不可并行)
        self.task_left_arm_turn_off_machine = task.Task_manipulation(task.Task_type.Task_manipulation.Turn_off_coffee_machine,None,utilis.Device_id.LEFT,\
            copy.deepcopy(grip_everything.left_arm_machine_turn_off_pre), arm_move_method = arm.ArmMoveMethod.Z_XY,click_length=-10)

        #  将右臂(拿水)抬到指定位置
        self.task_right_arm_water_delivery = task.Task_manipulation(task.Task_type.Task_manipulation.Deliever_cup,None,utilis.Device_id.RIGHT,\
            grip_everything.right_arm_cup_delivery,arm.GripMethod.DONTCANGE, arm_move_method = arm.ArmMoveMethod.XYZ)

    
        #  将饮料臂放到指定位置后松开(不可并行)
        self.task_right_arm_placement_cup = task.Task_manipulation(task.Task_type.Task_manipulation.Lossen_cup,None,utilis.Device_id.RIGHT,\
            grip_everything.right_arm_cup_placement,arm.GripMethod.OPEN, arm_move_method = arm.ArmMoveMethod.XYZ)
        




def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('template')
    global grip_everything
    grip_everything      = Grip_everything()
    manipulator_actuator = Manipulator_actuator()
    tasks = Manipulator_tasks()
    
    # manipulator_actuator.run(tasks.task_arms_idle)
    manipulator_actuator.run(tasks.task_right_arm_to_rec_cup_pre)
    manipulator_actuator.run(tasks.task_left_arm_to_rec_coffee_machine_turn_on)
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("grip_everything")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
