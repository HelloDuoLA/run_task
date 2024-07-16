#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import datetime
import os
import cv2
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")


LOGDIR = package_path + "/log/"
SUBDIR = ["orders","tasks","STag_result","images","finished_tasks","empirical_value"]

for sub_dir in SUBDIR:
    if not os.path.exists(LOGDIR + sub_dir):
        os.makedirs(LOGDIR + sub_dir)


def get_current_time():
    # 获取当前时间的秒数
    time_sec = rospy.Time.now().to_sec()
    # 将秒数转换为datetime对象
    time_datetime = datetime.datetime.fromtimestamp(time_sec)
    # 格式化时间为月日时分秒格式
    formatted_time = time_datetime.strftime('%m-%d %H:%M:%S')
    return formatted_time

# 记录任务信息
def log_tasks_info(context,filename,middle_name="tasks"):
    file_path = f"{LOGDIR}/{middle_name}/{filename}"
    rospy.loginfo(f"{rospy.get_name()} add log f{file_path}")
    with open(file_path, 'a') as file:
        file.write(get_current_time() + '\n')
        file.write("\r\n" + "-"*50 + "\r\n")
        file.write(str(context))
        file.write("\r\n" + "-"*50 + "\r\n")
        file.write("\n\n")
        
# 记录任务信息
def log_update_tasks_info(context,filename,middle_name="tasks"):
    file_path = f"{LOGDIR}/{middle_name}/{filename}.log"
    rospy.loginfo(f"{rospy.get_name()} add log f{file_path}")
    with open(file_path, 'w') as file:
        file.write(get_current_time() + '\n')
        file.write("\r\n" + "-"*50 + "\r\n")
        file.write(str(context))
        file.write("\r\n" + "-"*50 + "\r\n")
        file.write("\n\n")

# 把新增任务列表写入
def log_add_tasks_info(context):
    log_tasks_info(context,"add_tasks_info.log")

# 将完成任务信息写入
# def log_update_finish_tasks_info(context):
#     log_update_tasks_info(context,"finish_tasks_info.log")
    
# 将修改任务信息写入
def log_modify_tasks_info(context):
    log_tasks_info(context,"modify_tasks_info.log")

# 把新增订单写入
def log_new_order_info(context,):
    log_tasks_info(context,"new_order_info.log","orders")

# 保存图片
def log_write_image(image_name,image):
    rospy.loginfo(f"file path {f'{LOGDIR}/images/{image_name}'}")
    cv2.imwrite(f'{LOGDIR}/images/{image_name}', image)

# 保存 stag result 结果
def log_stag_result(filename,context):
    with open(f"{LOGDIR}/STag_result/{filename}", 'a') as file:
        file.write(context)

# 保存姿势经验值
def log_pose_empirical_value(context,pose_name):
    with open(f"{LOGDIR}/empirical_value/{pose_name}.log", 'a') as file:
        file.write(context)
        
# 保存左臂抓零食经验值
def log_empirical_value_left_arm_grip_snack(context):
    pose_name = "left_arm_grip_snack"
    log_pose_empirical_value(str(context),pose_name)

# 保存右臂抓零食经验值
def log_empirical_value_right_arm_grip_snack(context):
    pose_name = "right_arm_grip_snack"
    log_pose_empirical_value(context,pose_name)
    
# 保存左臂放零食经验值
def log_empirical_value_left_arm_lossen_snack(context):
    pose_name = "left_arm_lossen_snack"
    log_pose_empirical_value(context,pose_name)

# 保存右臂放零食经验值
def log_empirical_value_right_arm_lossen_snack(context):
    pose_name = "right_arm_lossen_snack"
    log_pose_empirical_value(context,pose_name)
    
# 保存左臂抓容器经验值
def log_empirical_value_left_arm_grip_container(context):
    pose_name = "left_arm_grip_container"
    log_pose_empirical_value(context,pose_name)

# 保存右臂抓容器经验值
def log_empirical_value_right_arm_grip_container(context):
    pose_name = "right_arm_grip_container"
    log_pose_empirical_value(context,pose_name)
    
# 保存左臂打开咖啡机经验值
def log_empirical_value_left_arm_turn_on_machine(context):
    pose_name = "left_arm_turn_on_machine"
    log_pose_empirical_value(context,pose_name)
    
# 保存左臂关闭咖啡机经验值
def log_empirical_value_left_arm_turn_off_machine(context):
    pose_name = "left_arm_turn_off_machine"
    log_pose_empirical_value(context,pose_name)

# 保存右臂抓杯子经验值
def log_empirical_value_right_arm_grip_cup(context):
    pose_name = "right_arm_grip_cup"
    log_pose_empirical_value(context,pose_name)
    
# 保存右臂加水经验值
def log_empirical_value_right_arm_water_cup(context):
    pose_name = "right_arm_water_cup"
    log_pose_empirical_value(context,pose_name)