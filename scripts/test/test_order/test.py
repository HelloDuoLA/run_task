#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import sys
sys.path.insert(0,"/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/scripts")

import order
import rospy


order_info = order.Order()
order_info.set_order_operation(order.Order.Operation.ADD)
order_info.set_table_id(1)

snack  = order.Snack(order.Snack.Snack_id.YIDA,1)
snack2 = order.Snack(order.Snack.Snack_id.CHENPIDAN,3)
drink  = order.Drink(order.Drink.Drink_id.COFFEE,1)

order_info.add_snack(snack)
order_info.add_snack(snack2)
order_info.add_drink(drink)

# print(order_info)
# print(order_info.to_dict())
# print(snack2.to_dict())
# print(order_info.to_dict())
order_info.to_yaml("/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/scripts/test/test_order/order/order1.yaml")
order_info.to_json("/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/scripts/test/test_order/order/order1.json")



