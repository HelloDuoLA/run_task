#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import os
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis

import assign_tasks
import run_task.msg as msg
import order

def ensure_directory_exists(path):
    if not os.path.exists(path):
        os.makedirs(path)



def test_tasks_before_grasp_snack():
    pass


def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('test_assgin_order')
    
    path = './src/run_task/log/order'
    ensure_directory_exists(path)
    
    task_schedul = assign_tasks.Order_driven_task_schedul(None)
    
    order_info = order.Snack_list()

    snack  = order.Snack(order.Snack.Snack_id.YIDA,1)
    snack2 = order.Snack(order.Snack.Snack_id.CHENPIDAN,1)
    drink  = order.Drink(order.Drink.Drink_id.COFFEE,1)

    order_info.add_snack(snack)
    order_info.add_snack(snack2)
    
    
    tasks_before_grasp_snack =task_schedul.create_tasks_before_grasp_snack(order_info)
    
    with open(f"{path}/order.txt", 'w') as file:
        file.write(tasks_before_grasp_snack)
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(0.5)
    
    
    while not rospy.is_shutdown():
        rospy.loginfo("test_assgin_order")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
