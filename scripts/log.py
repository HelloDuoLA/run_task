#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import datetime
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")


log_dir = package_path + "/log"

def get_current_time():
    # 获取当前时间的秒数
    time_sec = rospy.Time.now().to_sec()
    # 将秒数转换为datetime对象
    time_datetime = datetime.datetime.fromtimestamp(time_sec)
    # 格式化时间为月日时分秒格式
    formatted_time = time_datetime.strftime('%m-%d %H:%M:%S')
    return formatted_time

# 把新增任务列表写入
def log_add_tasks_info(context):
    rospy.loginfo(f"{rospy.get_name()} log_add_tasks_info ")
    part_filename = get_current_time() 
    file_path = log_dir  + "/add_tasks_info.log"
    with open(file_path, 'a') as file:
        file.write("\r\n\r\n\r\n")
        file.write(part_filename + '\n')
        file.write("\r\n" + "-"*50 + "\r\n")
        file.write(str(context))
        file.write("\r\n" + "-"*50 + "\r\n")

# 新的订单信息
def log_new_order_info(context):
    rospy.loginfo(f"{rospy.get_name()} log_new_order_info ")
    part_filename = get_current_time() 
    file_path = log_dir  + "/new_order_info.log"
    with open(file_path, 'a') as file:
        file.write("\r\n\r\n\r\n")
        file.write(part_filename + '\n')
        file.write("-"*50 + "\r\n")
        file.write(str(context))
        file.write("\r\n" + "-"*50 + "\r\n")