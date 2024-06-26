#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis

import run_task.msg as msg
import order

    

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('test_order')
    
    pub = rospy.Publisher(utilis.Topic_name.make_order,msg.OrderInfo,queue_size=10)

    order_info = order.Order()
    order_info.set_order_operation(order.Order.Operation.ADD)
    order_info.set_table_id(order.Table_id.LEFT)

    snack  = order.Snack(order.Snack.Snack_id.YIDA,1)
    snack2 = order.Snack(order.Snack.Snack_id.CHENPIDAN,1)
    drink  = order.Drink(order.Drink.Drink_id.COFFEE,1)

    order_info.add_snack(snack)
    order_info.add_snack(snack2)
    order_info.add_drink(drink)
    msg_order = order_info.to_msg()
    rospy.loginfo(f"node name:{rospy.get_name()} msg:{msg_order}")
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(0.5)
    
    """
    测试输出区
    """
    # rospy.loginfo(f"{order.Order.Operation(0)}")
    
    # obj_3 =order.Order.Operation(3)
    # rospy.loginfo(f"obj_3 : {obj_3}")
    
    
    while not rospy.is_shutdown():
        rospy.loginfo("test_order")
        pub.publish(msg_order)
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
