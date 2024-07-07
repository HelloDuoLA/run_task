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
import task

# 模板文件

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('test_camera_topic')
    
    pub = rospy.Publisher(utilis.Topic_name.image_recognition_request,msg.ImageRecRequest,queue_size=10)
    #3.组织消息
    task_index = 0
    request = msg.ImageRecRequest()
    # request.camera_id = utilis.Device_id.LEFT.value
    request.camera_id = utilis.Device_id.LEFT_RIGHT.value
    request.task_type = task.Task_type.Task_image_rec.SNACK.value
    request.snacks = [1,8]
    
    rospy.loginfo(f"request {request}")

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        request.task_index = task_index
        task_index += 1
        if task_index > 100:
            task_index = 0
        rospy.loginfo("test_camera_topic")
        pub.publish(request)
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
