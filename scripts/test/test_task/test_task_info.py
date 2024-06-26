#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import datetime
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import task
import assign_tasks


# 模板文件

def talker():
    rospy.init_node('test_task_info')

    # rospy.loginfo(f"time {rospy.Time.now().secs} ")
    # 获取当前时间的秒数
    now = rospy.Time.now().to_sec()
    rospy.loginfo(f"Current Time: {now}")
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)
    # task_navigation = task.Task_navigation()
    system = assign_tasks.System()
    task_left_arm_to_rec_snack = task.Task_manipulation(task.Task_type.Task_manipulation.Move, None, utilis.Device_id.LEFT, system.constant_config.arm_anchor_point.left_arm_snack_rec)
    task_left_arm_to_rec_snack.parallel = task.Task.Task_parallel.ALL
    rospy.loginfo(task_left_arm_to_rec_snack)
    # rospy.loginfo(task_left_arm_to_rec_snack.task_type)
    # a = task.Task_type(task.Task_type.Task_manipulation.Move)
    # rospy.loginfo(a)
        

    while not rospy.is_shutdown():
        rospy.loginfo("test_task_info")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
