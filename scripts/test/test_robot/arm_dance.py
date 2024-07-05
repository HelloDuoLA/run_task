#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import copy
import actionlib
import run_task.msg as msg

rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis

from std_msgs.msg import String

import arm
import task
import robot


left_arm_idle_state  = True
right_arm_idle_state = True


# 机械臂跳舞

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('arm_dance')
    task_list = []
    
    left_arm_idle                = constant_config_to_arm_anchor_pose("LeftArmIdle",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_container_rec       = constant_config_to_arm_anchor_pose("LeftArmContainerRec",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_snack_rec           = constant_config_to_arm_anchor_pose("LeftArmSnackRec",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_snack_placement     = constant_config_to_arm_anchor_pose("LeftArmSnackPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    left_arm_container_delivery  = constant_config_to_arm_anchor_pose("LeftArmContainerDelivery",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    left_arm_container_placement = constant_config_to_arm_anchor_pose("LeftArmContainerPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    
    right_arm_idle                = constant_config_to_arm_anchor_pose("RightArmIdle",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_container_rec       = constant_config_to_arm_anchor_pose("RightArmContainerRec",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_snack_rec           = constant_config_to_arm_anchor_pose("RightArmSnackRec",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_snack_placement     = constant_config_to_arm_anchor_pose("RightArmSnackPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_container_delivery  = constant_config_to_arm_anchor_pose("RightArmContainerDelivery",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_container_placement = constant_config_to_arm_anchor_pose("RightArmContainerPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    
    right_arm_cup_grab            = constant_config_to_arm_anchor_pose("CupGrab",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_rec             = constant_config_to_arm_anchor_pose("CupRec",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_cup_rec_before      = constant_config_to_arm_anchor_pose("CupRecBefore",arm.PoseType.ANGLE,utilis.Device_id.RIGHT)
    right_arm_cup_water           = constant_config_to_arm_anchor_pose("CupWater",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_delivery        = constant_config_to_arm_anchor_pose("CupDelivery",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_placement       = constant_config_to_arm_anchor_pose("CupPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)

    left_arm_coffee_machine_rec           = constant_config_to_arm_anchor_pose("CoffeeMachineRec",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_coffee_machine_open_prepare  = constant_config_to_arm_anchor_pose("CoffeeMachineOpenPrepare",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    left_arm_coffee_machine_close_prepare = constant_config_to_arm_anchor_pose("CoffeeMachineClosePrepare",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    
    # 识别容器 1 
    task_arms_to_rec_contianer = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_container_rec,right_arm_container_rec])
    task_list.append(task_arms_to_rec_contianer)
    
    # 识别零食 2
    task_arms_to_rec_snack     = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_snack_rec,right_arm_snack_rec])
    task_list.append(task_arms_to_rec_snack)
    
    # 抓取容器
    # TODO:
    # task_arms_to_
    
    # 收紧抓爪
    # TODO:
    
    # 转移容器 3
    left_arm_container_delivery.set_base_coords_y(200)
    right_arm_container_delivery.set_base_coords_y(-200)
    task_arms_container_delivery = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_container_delivery ,right_arm_container_delivery])
    task_list.append(task_arms_container_delivery)
    
    # 放置容器 4
    left_arm_container_placement.set_base_coords_y(200)
    right_arm_container_placement.set_base_coords_y(-200)
    task_arms_container_placement = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_container_placement ,right_arm_container_placement])
    task_list.append(task_arms_container_placement)
    
    # 左,右臂识别杯子前摇
    task_arms_to_rec_cup_machine_before = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_idle, right_arm_cup_rec_before],[robot.manipulation_status.clamp.status.CLOSE,robot.manipulation_status.clamp.status.CLOSE]) 
    task_list.append(task_arms_to_rec_cup_machine_before)
    
    # 识别杯子_machine
    task_arms_to_rec_cup_machine    = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_coffee_machine_rec ,right_arm_cup_rec])
    task_list.append(task_arms_to_rec_cup_machine)
        
    # 夹取杯子 
    right_arm_cup_grab.set_base_coords_y(-200)
    right_arm_cup_grab.set_base_coords_x(400)
    task_right_arm_cup_grab         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_grab],robot.manipulation_status.clamp.status.OPEN)
    task_right_arm_cup_grab.set_clamp_first
    task_list.append(task_right_arm_cup_grab)
    
    # 夹取杯子关闭夹子
    task_right_arm_cup_grab_close    = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_grab],robot.manipulation_status.clamp.status.CLOSE)
    task_right_arm_cup_grab_close.set_clamp_first
    task_list.append(task_right_arm_cup_grab_close)
    
    # 开启机器
    left_arm_coffee_machine_open_prepare.set_base_coords_x(300)
    left_arm_coffee_machine_open_prepare.set_base_coords_y(200)
    left_arm_coffee_machine_open_prepare.set_base_coords_z(400)
    task_left_arm_coffee_machine_open_prepare = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [left_arm_coffee_machine_open_prepare])
    task_list.append(task_left_arm_coffee_machine_open_prepare)
    
    
    left_arm_coffee_machine_open = copy.deepcopy(left_arm_coffee_machine_open_prepare)
    left_arm_coffee_machine_open.arm_pose[2] = left_arm_coffee_machine_open.arm_pose[2] + 20
    task_left_arm_coffee_machine_open         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [left_arm_coffee_machine_open])
    task_list.append(task_left_arm_coffee_machine_open)
    
    # 杯子打水
    right_arm_cup_water.set_base_coords_x(300)
    right_arm_cup_water.set_base_coords_y(-50)
    task_right_arm_cup_water         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_water])
    task_list.append(task_right_arm_cup_water )
    
    # 关闭机器
    left_arm_coffee_machine_close_prepare.set_base_coords_x(350)
    left_arm_coffee_machine_close_prepare.set_base_coords_y(200)
    left_arm_coffee_machine_close_prepare.set_base_coords_z(450)
    task_left_arm_coffee_machine_close_prepare = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [left_arm_coffee_machine_close_prepare])
    task_list.append(task_left_arm_coffee_machine_close_prepare)
    
    # 向上拨一拨12
    left_arm_coffee_machine_close = copy.deepcopy(left_arm_coffee_machine_close_prepare)
    left_arm_coffee_machine_close.arm_pose[2] = left_arm_coffee_machine_close.arm_pose[2] - 20
    task_left_arm_coffee_machine_close         = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, [left_arm_coffee_machine_close])
    task_list.append(task_left_arm_coffee_machine_close)
    
    # 移动杯子13
    right_arm_cup_delivery.set_base_coords_z(210)
    task_right_arm_cup_delivery = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_delivery])
    task_list.append(task_right_arm_cup_delivery)
    
    # 放置杯子 14
    right_arm_cup_placement.set_base_coords_z(210)
    task_right_arm_cup_placement = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.RIGHT, [right_arm_cup_placement], robot.manipulation_status.clamp.status.OPEN)
    task_list.append(task_right_arm_cup_placement)
    
    # 回归空闲 15
    task_arms_to_idle       = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT_RIGHT, [left_arm_idle ,right_arm_idle],[robot.manipulation_status.clamp.status.CLOSE,robot.manipulation_status.clamp.status.CLOSE])
    task_list.append(task_arms_to_idle)
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    with open('/home/elephant/xzc_code/ros_ws/src/run_task/scripts/test/test_robot/arm_dance_output.txt', 'w') as f:
        for i in range(len(task_list)):
            f.write(f"id {i}\n")
            f.write(f"{task_list[i]}\n")
    
    task_index = 0
    manipulator_actuator = Manipulator_actuator()
    global right_arm_idle_state ,left_arm_idle_state 
    rospy.loginfo("111111111")
    while not rospy.is_shutdown():
        if  task_index < len(task_list):
            if task_list[task_index].arm_id == utilis.Device_id.LEFT and left_arm_idle == True:
                left_arm_idle_state = False
                manipulator_actuator.run(task_list[task_index])
                task_index += 1
            elif task_list[task_index].arm_id == utilis.Device_id.RIGHT and right_arm_idle_state == True:
                right_arm_idle_state = False
                manipulator_actuator.run(task_list[task_index])
                task_index += 1
            elif task_list[task_index].arm_id == utilis.Device_id.LEFT_RIGHT and left_arm_idle_state == True and right_arm_idle_state == True:
                left_arm_idle_state = False
                right_arm_idle_state = False
                manipulator_actuator.run(task_list[task_index])
                task_index += 1
        rospy.loginfo("arm")
        # 按照设定的频率延时
        rate.sleep()
        


def constant_config_to_arm_anchor_pose(anchor_point_name,type:arm.PoseType,arm_id:utilis.Device_id):
    arm_pose = arm.Arm_pose()
    if type == arm.PoseType.ANGLE:
        arm_pose.arm_pose = rospy.get_param(f'~{anchor_point_name}/angles')
    elif type == arm.PoseType.BASE_COORDS:
        arm_pose.arm_pose = rospy.get_param(f'~{anchor_point_name}/coords')

    arm_pose.type_id  = type
    arm_pose.arm_id   = arm_id
    rospy.loginfo(f"{anchor_point_name} : type {type}, pose {arm_pose.arm_pose}")
    return arm_pose        


# 机械臂执行器 
class Manipulator_actuator():
    def __init__(self):
        self.left_arm_ac  = actionlib.SimpleActionClient(utilis.Topic_name.left_arm_action,  msg.MoveArmAction)
        self.right_arm_ac = actionlib.SimpleActionClient(utilis.Topic_name.right_arm_action, msg.MoveArmAction)
        # TODO:调试需要,暂时注释
        # self.left_arm_ac.wait_for_server()
        # self.right_arm_ac.wait_for_server()
        self.running_tasks_manager = task.Task_manager_in_running() # 正在执行的任务管理器
    
    # 运行
    def run(self, manipulation_task:task.Task_manipulation):
        rospy.loginfo("running")
        # 加在运行序列中
        # task_index = self.running_tasks_manager.add_task(manipulation_task)
        
        # 任务开始
        # manipulation_task.start_time()
        task_index = manipulation_task.task_index
        # 单臂
        if manipulation_task.arm_id == utilis.Device_id.LEFT:
            # 设置action 目标
            goal                      = msg.MoveArmGoal()
            goal.task_index           = task_index
            print(task_index)
            goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0]
            print(f"manipulation_task.target_arms_pose[0] {manipulation_task.target_arms_pose[0]}")
            goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            goal.grasp_first          = manipulation_task.clamp_first
            goal.grasp_speed          = manipulation_task.clamp_speed
            goal.arm_id               = utilis.Device_id.LEFT.value
            self.left_arm_ac.send_goal(goal,self.done_callback,self.active_callback,self.feedback_callback)
        
        elif manipulation_task.arm_id == utilis.Device_id.RIGHT:
            # 设置action 目标
            goal                      = msg.MoveArmGoal()
            goal.task_index           = task_index
            goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[0]
            goal.arm_pose.type_id     = manipulation_task.target_arms_pose[0].type_id.value
            goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[0].arm_id.value
            goal.grasp_flag           = manipulation_task.target_clamps_status[0].value
            goal.grasp_first          = manipulation_task.clamp_first
            goal.grasp_speed          = manipulation_task.clamp_speed
            goal.arm_id               = utilis.Device_id.RIGHT.value
            self.right_arm_ac.send_goal(goal,self.done_callback,self.active_callback,self.feedback_callback)
            
        elif manipulation_task.arm_id == utilis.Device_id.LEFT_RIGHT:
            rospy.loginfo("utilis.Device_id.LEFT_RIGHT")
            left_goal              = msg.MoveArmGoal()
            right_goal             = msg.MoveArmGoal()
            for i in range(2):
                if manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.LEFT:
                    left_goal.task_index           = task_index
                    left_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i].arm_pose
                    left_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    left_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    left_goal.grasp_first          = manipulation_task.clamp_first
                    left_goal.grasp_speed          = manipulation_task.clamp_speed
                    left_goal.arm_id               = utilis.Device_id.LEFT.value
                    
                elif manipulation_task.target_arms_pose[i].arm_id == utilis.Device_id.RIGHT:
                    right_goal.task_index           = task_index
                    right_goal.arm_pose.arm_pose    = manipulation_task.target_arms_pose[i].arm_pose
                    right_goal.arm_pose.type_id     = manipulation_task.target_arms_pose[i].type_id.value
                    right_goal.arm_pose.arm_id      = manipulation_task.target_arms_pose[i].arm_id.value
                    right_goal.grasp_first          = manipulation_task.clamp_first
                    right_goal.grasp_speed          = manipulation_task.clamp_speed
                    left_goal.arm_id                = utilis.Device_id.RIGHT.value
            rospy.loginfo("send goal!!!")
            rospy.loginfo(left_goal)
            rospy.loginfo(right_goal)
            self.left_arm_ac.send_goal(left_goal,self.done_callback,self.active_callback,self.feedback_callback)
            self.right_arm_ac.send_goal(right_goal,self.done_callback,self.active_callback,self.feedback_callback)


    # 完成回调
    @staticmethod
    def done_callback(status, result:msg.MoveArmResult):
        global left_arm_idle_state,right_arm_idle_state
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator done. status:{status} result:{result}")
        if result.arm_id == utilis.Device_id.LEFT:
            left_arm_idle_state = True
        elif result.arm_id == utilis.Device_id.RIGHT:
            right_arm_idle_state = True
        elif result.arm_id == utilis.Device_id.LEFT_RIGHT:
            left_arm_idle_state = True
            right_arm_idle_state = True

    # 激活回调
    @staticmethod
    def active_callback():
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator active")
    
    # 反馈回调
    @staticmethod
    def feedback_callback(feedback:msg.MoveArmFeedback):
        rospy.loginfo(f"node: {rospy.get_name()}, manipulator feedback. {feedback}")


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
