#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys

path = os.path.abspath(".")
sys.path.insert(0,path + "/src/run_task/scripts")



from std_msgs.msg import String

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('test_service_client')

    
    client = rospy.ServiceProxy("ImageRec",ImageRec)
    client.wait_for_service()
    req = ImageRecRequest()
    req.task_type = 0
    resp = client(req)
    resp = client.call(req)
    rospy.loginfo(resp)
    rospy.loginfo("snack count = %s",len(resp.snack_type))
    
    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("test_service_client")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
