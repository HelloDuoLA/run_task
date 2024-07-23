import pyaudio
import time
from speech_recognition_module import SpeechRecognizer
from ai_interaction_module import get_ai_response_as_dict, chat_with_ai
from speech_synthesis_module import tts

# 全局对话计数器
global_count = 1
start_timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')

def list_microphones():
    """列出系统中连接的麦克风设备"""
    p = pyaudio.PyAudio()
    print("当前连接的麦克风设备：")
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info["maxInputChannels"] > 0:
            print(f"{i}: {device_info['name']}")
    p.terminate()

def get_valid_microphone_index():
    """提示用户选择有效的麦克风设备编号"""
    p = pyaudio.PyAudio()
    device_count = p.get_device_count()
    p.terminate()

    while True:
        try:
            mic_index = int(input("请选择要使用的麦克风设备编号: "))
            if 0 <= mic_index < device_count:
                return mic_index
            else:
                print(f"无效的设备编号，请输入一个介于 0 和 {device_count - 1} 之间的数字。")
        except ValueError:
            print("无效输入，请输入一个数字。")

def prompt_user():
    """提示用户是否开始语音识别"""
    while True:
        time.sleep(0.5)
        choice = input("是否开始语音识别？ (y/n): ").strip().lower()
        if choice in ['y', 'n']:
            return choice == 'y'
        else:
            print("无效输入，请输入 'y' 或 'n'。")

def voice_to_json(APPID, APIKey, APISecret, messages, mic_index):
    global global_count
    recognizer = SpeechRecognizer(APPID, APIKey, APISecret, global_count, mic_index)
    print("开始语音识别...")
    recognizer.start_recognition()
    recognizer.closed_event.wait()
    recognized_text = recognizer.stop_recognition()
    time.sleep(1)
    print(f"识别到的文字: {recognized_text}")

    if recognized_text:
        print("将识别的文字传递给AI处理...")
        messages.append({"role": "user", "content": recognized_text})
        response_files, response_dict, response = get_ai_response_as_dict(messages,
                                                count=global_count, start_timestamp=start_timestamp)
        messages.append({"role": "assistant", "content": response})
        if response_files:
            print("AI处理完成并生成了文件。")
        else:
            print("AI处理完成, 但未生成文件。")
            # 调用语音合成函数
            tts(APPID, APIKey, APISecret, response)
        
        global_count += 1  # 每次成功识别后递增计数
        return True, response_files, response_dict, messages
    else:
        print("未识别到有效文字。")
        return False, None, None, messages

def text_to_speech(APPID, APIKey, APISecret, response):
    """调用语音合成函数"""
    tts(APPID, APIKey, APISecret, response)


def main():
    # global global_count
    APPID = 'ec798696'
    APIKey = '079797c2b651aada7573f75eb2ca1955'
    APISecret = 'YzU5ZWM2NjJmOGY1ODQ0ZmM3M2ViZWEy'

    use_default_microphone = True  # 设置为 True 时，直接使用默认麦克风设备

    initial_prompt = (
        "给我生成一份json文件，内容包括order_operation、table_id、snacks、drinks、other_items六个对象。"
        "order_operation包括0、1、2、3，分别代表ADD、DELETE、MODIFT、CHECK，"
        "执行一次送货的操作就是ADD；table_id包括1、2，其中办公桌一赋值为1，办公桌二赋值为2；"
        "snacks又包括需要搬运的不同的零食对象，其中1代表益达口香糖、2代表果蔬果冻、3代表C酷果冻、"
        "4代表伊利每益添乳酸菌、5代表金津陈皮丹，每种零食又有不同的值，代表有多少个；"
        "drinks又包括需要搬运的不同饮料对象，其中1代表咖啡，每种饮料又有不同的值，代表有多少个。"
        "我的送货任务是：假如你是机器人，我对机器人说一段话，将一个C酷果冻放到办公桌一上，然后你要识别出其中的关键信息，"
        "一个、C酷果冻、办公桌一，并按照之前跟你描述的json对象格式一一填入，如果某个对象没有，都要一起返回，"
        "对象的值填0，最终返回一个完整的json内容，"
        "如果识别到的对象不在snacks和drinks的种类中，则视为是另类物品，不作处理，返回的json对象中不显示出来。"
        "要灵活变通一点，因为我是通过说话语音识别得到后的文字输入给你的。"
        "json内容中不要添加任何说明文字，只需要对应的json对象以及值。"
        "就像上述的一样，识别到C酷果冻，那么在snacks对象中的3对应值是1，其他没有识别到的也要显示出来，"
        "值设为0，比如snacks中的1、2、4、5，drinks中的1都要一并写出来，且值设为0,"
        "假如我说的只是拿一杯咖啡到办公桌一上，那么你也要把snacks中的1、2、3、4、"
        "5对象写出来并值都设为0，这点非常重要一定要写出来，"
        "如果识别到的内容都不在跟你说的种类中，那么就不予理会直接忽略，比如我说的是拿一个红薯到办公桌二"
        "上或者其他snacks和drinks中没有提到过的种类的物品，这时你应该能提取到关键信息红薯和办公桌二，"
        "但由于红薯不在我预设的snacks和drinks种类中，直接忽略这一个订单,不要再生成任何json内容了，"
        "如果我的指令是：拿一个C酷果冻和两个益达口香糖到办公桌一上，你要生成如下的json内容："
        "{\n"
        "    \"order_operation\": 0,\n"
        "    \"table_id\": 1,\n"
        "    \"snacks\": {\n"
        "        \"1\": 2,\n"
        "        \"2\": 0,\n"
        "        \"3\": 1,\n"
        "        \"4\": 0,\n"
        "        \"5\": 0\n"
        "    },\n"
        "    \"drinks\": {\n"
        "        \"1\": 0\n"
        "    }\n"
        "}\n"
        "要像上面那样，snacks中的对象\"1\"填入值为1,对象\"5\"填入值为1,其他均为0。"
        "始终按照种类顺序1、2、3、4、5的格式生成json对象,"
        "不要将识别到的种类先排到前面，始终按照1、2、3、4、5的json对象顺序排列。"
        "一条指令中识别到两个零食时都要提取出来填入同一个json对象相应的内容中。"
        "且任务中规定一个桌子只能放零食，另一个桌子只能放咖啡，所以零食一定在同一个桌子上，也就是在同一个json对象中。"
        "当输入指令是雪碧、可乐、奶茶等其他不是咖啡的饮品时，你只需要作出以下回答："
        "不好意思，没有（输入指令的物品），饮品只有咖啡可供选择。回答到此为止，回答的字数限制在10个字以内，越短越好，"
        "不需要再生成json内容了。"
        "不要生成其他任何无关json对象的文字内容，只需要生成上述的json内容,不需要任何有关生成内容的解释,"
        "尽可能快的生成出json内容。"
        "以上内容都是我给你预设的内容，这一次对话无需你作任何回答，接下来我跟你说指令时你再按要求生成json内容或其他回答。"
    )

    messages = [{"role": "user", "content": initial_prompt}]
    # 获取初始AI回答（不处理输出或保存）
    initial_response = chat_with_ai(messages)
    messages.append({"role": "assistant", "content": initial_response})
    if initial_response:
        print("系统初始化完成，等待语音识别...")

    # 获取麦克风设备编号
    if not use_default_microphone:
        list_microphones()
        mic_index = get_valid_microphone_index()
    else:
        mic_index = None  # 使用默认麦克风设备

    # 开始语音对话
    while True:
        try:
            if not prompt_user():
                print("已取消本次语音识别。")
                break
            else :
                continue_recognition, _, _, messages = voice_to_json(APPID, APIKey, APISecret,
                                                            messages, mic_index, )
        except KeyboardInterrupt:
            print("程序已终止。")
            break

huanYingYu = "你好, 送餐机器人为您服务"
def test_tts():
    APPID = 'ec798696'
    APIKey = '079797c2b651aada7573f75eb2ca1955'
    APISecret = 'YzU5ZWM2NjJmOGY1ODQ0ZmM3M2ViZWEy'
    response = "你好"
    tts(APPID, APIKey, APISecret, huanYingYu)


if __name__ == "__main__":
    # test_tts()
    main()