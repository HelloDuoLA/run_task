#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import rospkg
import std_srvs.srv as std_srvs
import std_msgs.msg  as std_msgs
import time

# 自定义包
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
sys.path.insert(0,package_path + "/scripts/asr_module")
import run_task.msg as msg
import task
import utilis
import robot
import order
import log
import asr_module

from config_loader import APPID, APIKey, APISecret

# 语音识别节点
# TODO:待完成功能

class Asr_node():
    def __init__(self) -> None:
        self.messages = []  # 初始化空对话记录列表
        self.use_default_microphone = True
        # 获取麦克风设备编号
        if not self.use_default_microphone:
            asr_module.list_microphones()
            self.mic_index = asr_module.get_valid_microphone_index()
        else:
            self.mic_index = None  # 使用默认麦克风设备
            
        # 订单发布句柄
        self.order_pub = rospy.Publisher(utilis.Topic_name.make_order,msg.OrderInfo,queue_size=10)
        

    def run(self):
        try:
            # 欢迎语
            self.say_welcome()
            while True:

                continue_recognition, response_files, response_dict, self.messages = asr_module.voice_to_json(APPID, APIKey, APISecret,
                                                        self.messages, self.mic_index)
                if response_files:
                    rospy.loginfo("语音识别成功")
                    # 处理json
                    order_info  = msg.OrderInfo()
                    order_info.order_operation = response_dict.get("order_operation", 0)
                    order_info.table_id        = response_dict.get("table_id", 1)
                    
                    # 转换一下
                    if order_info.table_id == 1:
                        order_info.table_id = 2
                    else:
                        order_info.table_id = 1
                    snack_list = []
                    snack_all_count = 0
                    for snack_id, count in response_dict["snacks"].items():
                        if count != 0:
                            snack = msg.SnackIDWithCount()
                            snack.snack_id = int(snack_id)
                            snack.count = count
                            snack_list.append(snack)
                            snack_all_count += count
                    
                    drink_list = []
                    drink_count = 0
                    for drink_id, count in response_dict["drinks"].items():
                        if count != 0:
                            drink = msg.DrinkIDWithCount()
                            drink.drink_id = int(drink_id)
                            drink.count = count
                            drink_list.append(drink)
                            drink_count += count
                        
                    order_info.snacks = snack_list
                    order_info.drinks = drink_list
                    if drink_count == 0 and snack_all_count == 0:
                        rospy.loginfo("没有识别到任何物品")
                        self.say_again()
                        time.sleep(1)
                        continue

                    # 发布订单数据
                    self.order_pub.publish(order_info)
                    self.say_do_service()
                    rospy.loginfo(f"{order_info}")
                    break
                elif continue_recognition == False:  # 没有识别到文字
                    rospy.loginfo("语音识别失败")
                    self.say_again()
                    time.sleep(1)
        except KeyboardInterrupt:
            print("程序已终止。")
    
    # 文字转语音
    def text_to_speech(self,content):
        asr_module.text_to_speech(APPID, APIKey, APISecret, content)
    
    # 欢迎语
    def say_welcome(self):
        huanyingyu = "送餐机器人为你服务"
        self.text_to_speech(huanyingyu)
        
    # 欢迎语
    def say_do_service(self):
        content = "好的, 马上为你送达"
        self.text_to_speech(content)
    
    # 再来一次 
    def say_again(self):
        content = "不好意思, 能再说一次吗?"
        self.text_to_speech(content)

# 准备完成服务
def doPrepareReq(request):
    print("Received an empty service request")
    # 这里执行你需要的操作
    return std_srvs.EmptyResponse()  # 返回空响应

# 响应语音识别请求
def doAsrReq(request):
    asr_node.run()

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('asr')

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(0.1)
    global asr_node
    asr_node = Asr_node()
    
    # 语音识别请求话题
    sub_asr_request = rospy.Subscriber(utilis.Topic_name.asr_request,std_msgs.Empty,doAsrReq,queue_size=10)
    
    # 初始化成功节点
    prepare_server = rospy.Service(utilis.Topic_name.asr_prepare_service,std_srvs.Empty,doPrepareReq)

    while not rospy.is_shutdown():
        rospy.loginfo("asr")
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    

# rostopic pub /asr_request std_msgs/Empty "{}"
