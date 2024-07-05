#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis

from std_msgs.msg import String

import arm

left_arm_idle  = True
right_arm_idle = True

# 机械臂跳舞

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('arm_dance')
    
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
    right_arm_cup_water           = constant_config_to_arm_anchor_pose("CupWater",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_elivery         = constant_config_to_arm_anchor_pose("CupDelivery",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)
    right_arm_cup_placement       = constant_config_to_arm_anchor_pose("CupPlacement",arm.PoseType.BASE_COORDS,utilis.Device_id.RIGHT)

    left_arm_coffee_machine_rec           = constant_config_to_arm_anchor_pose("CoffeeMachineRec",arm.PoseType.ANGLE,utilis.Device_id.LEFT)
    left_arm_coffee_machine_open_prepare  = constant_config_to_arm_anchor_pose("CoffeeMachineOpenPrepare",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    left_arm_coffee_machine_close_prepare = constant_config_to_arm_anchor_pose("CoffeeMachineClosePrepare",arm.PoseType.BASE_COORDS,utilis.Device_id.LEFT)
    
    
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
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


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
