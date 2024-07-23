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

# 语音识别节点
# TODO:待完成功能

class Asr_node():
    def __init__(self) -> None:
        self.APPID = 'ec798696'
        self.APIKey = '079797c2b651aada7573f75eb2ca1955'
        self.APISecret = 'YzU5ZWM2NjJmOGY1ODQ0ZmM3M2ViZWEy'
        self.use_default_microphone = True  # 设置为 True 时，直接使用默认麦克风设备

        self.initial_prompt = (
            "给我生成一份json文件，内容包括order_operation、table_id、snacks、drinks、other_items六个对象。"
            "order_operation包括0、1、2、3，分别代表ADD、DELETE、MODIFT、CHECK，"
            "执行一次送货的操作就是ADD；table_id包括1、2，其中一号桌赋值为1，二号桌赋值为2；"
            "snacks又包括需要搬运的不同的零食对象，其中1代表益达口香糖、2代表果冻、3代表伊利每益添乳酸菌、"
            "4代表金津陈皮丹，每种零食又有不同的值，代表有多少个；"
            "drinks又包括需要搬运的不同饮料对象，其中1代表咖啡，每种饮料又有不同的值，代表有多少个。"
            "我的送货任务是：假如你是机器人，我对你说一段指令，拿一个果冻到一号桌上，然后你要识别出其中的关键信息，"
            "一个、果冻、一号桌，并按照之前跟你描述的json对象格式一一填入，如果某个对象没有，都要一起返回，"
            "对象的值填0，最终返回一个完整的json内容，"
            "如果识别到的对象不在snacks和drinks的种类中，则视为是另类物品，不作处理，返回的json对象中不显示出来。"
            "要灵活变通一点，因为我是通过说话语音识别得到后的文字输入给你的，有相近读音的可以视为一个物品。"
            "json内容中不要添加任何说明文字，只需要对应的json对象以及值。"
            "就像上述的一样，识别到果冻，那么在snacks对象中的2对应值是1，其他没有识别到的也要显示出来，"
            "值设为0，比如snacks中的1、3、4，drinks中的1都要一并写出来，且值设为0,"
            "假如我说的只是拿一杯咖啡到一号桌上，那么你也要把snacks中的1、2、3、4"
            "对象写出来并值都设为0，这点非常重要一定要写出来，"
            "如果识别到的内容都不在跟你说的种类中，那么就不予理会直接忽略，比如我说的是拿一个红薯到二号桌"
            "上或者其他snacks和drinks中没有提到过的种类的物品，这时你应该能提取到关键信息红薯和二号桌，"
            "但由于红薯不在我预设的snacks和drinks种类中，直接忽略这一个订单,不要再生成任何json内容了，"
            "如果我的指令是：拿一个益达口香糖和一个果冻到一号桌上，你要生成如下的json内容："
            "{\n"
            "    \"order_operation\": 0,\n"
            "    \"table_id\": 1,\n"
            "    \"snacks\": {\n"
            "        \"1\": 1,\n"
            "        \"2\": 1,\n"
            "        \"3\": 0,\n"
            "        \"4\": 0\n"
            "    },\n"
            "    \"drinks\": {\n"
            "        \"1\": 0\n"
            "    }\n"
            "}\n"
            "要像上面那样，snacks中的对象\"1\"填入值为1,对象\"2\"填入值为1,其他均为0。"
            "始终按照种类顺序1、2、3、4的格式生成json对象,"
            "不要将识别到的种类先排到前面，始终按照1、2、3、4的json对象顺序排列。"
            "一条指令中识别到两种零食时都要提取出来填入同一个json对象相应的内容中。"
            "且任务中规定一个桌子只能放零食，另一个桌子只能放咖啡，所以零食一定在同一个桌子上，也就是在同一个json对象中。"
            "我们提供的饮品只有咖啡, 无论询问多少次,我们都只能提供咖啡"
            "所以当输入指令是雪碧、可乐、奶茶等其他不是咖啡的饮品时，你只需要作出以下回答："
            "不好意思，没有（输入指令的物品），饮品只有咖啡可供选择。回答到此为止，回答的字数限制在25个字以内，越短越好，"
            "不需要再生成json内容了。"
            "不要生成其他任何无关json对象的文字内容，只需要生成上述的json内容,不需要任何有关生成内容的解释,"
            "尽可能快的生成出json内容。"
            "以上内容都是我给你预设的内容，这一次对话无需你作任何回答，接下来我跟你说指令时你再按要求生成json内容或其他回答。"
        )
        

        self.messages = [{"role": "user", "content": self.initial_prompt}]
        # 获取初始AI回答（不处理输出或保存）
        initial_response = asr_module.chat_with_ai(self.messages)
        # 这个message的作用是什么
        self.messages.append({"role": "assistant", "content": initial_response})
        if initial_response:
            print("系统初始化完成，等待语音识别...")

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
                continue_recognition, response_files, response_dict,  self.messages = asr_module.voice_to_json(self.APPID, self.APIKey, self.APISecret,
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
        asr_module.text_to_speech(self.APPID, self.APIKey, self.APISecret, content)
    
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
