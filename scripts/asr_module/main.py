import pyaudio
import time
from speech_recognition_module import recognize_speech, SpeechRecognizer
from ai_interaction_module import get_ai_response_as_dict, chat_with_ai

# 全局对话计数器
global_count = 1
start_timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')

# TODO:返回包含默认麦克风设备的索引
def list_microphones():
    """列出系统中连接的麦克风设备"""
    p = pyaudio.PyAudio()
    print("当前连接的麦克风设备：")
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info["maxInputChannels"] > 0:
            print(f"{i}: {device_info['name']}")
    p.terminate()

# TODO:增加一个参数,若该参数为真, 则使用默认麦克风
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

# TODO:修改为使用服务开启语音识别
def prompt_user():
    """提示用户是否开始语音识别"""
    while True:
        choice = input("是否开始语音识别？ (y/n): ").strip().lower()
        if choice in ['y', 'n']:
            return choice == 'y'
        else:
            print("无效输入，请输入 'y' 或 'n'。")


def voice_to_json(APPID, APIKey, APISecret, messages, duration=10, mic_index=None):
    global global_count

    if not prompt_user():
        print("已取消本次语音识别。")
        return False, None, None, messages

    print("开始语音识别...")
    recognizer = SpeechRecognizer(APPID, APIKey, APISecret)   # TODO成为全局变量
    recognizer.start_recognition()
    time.sleep(10)  # 设定识别的持续时间
    recognized_text = recognizer.stop_recognition()
    # recognized_text = recognize_speech(global_count, start_timestamp, duration, mic_index)
    print(f"识别到的文字: {recognized_text}")

    if recognized_text:
        print("将识别的文字传递给AI处理...")
        messages.append({"role": "user", "content": recognized_text})
        # TODO:response_dict 是我想要的嘛
        response_files, response_dict, response = get_ai_response_as_dict(messages, count=global_count, start_timestamp=start_timestamp)
        messages.append({"role": "assistant", "content": response})
        if response_files:
            global_count += 1  # 每次成功识别并生成文件后递增计数
            print("AI处理完成并生成了文件。")
        else:
            print("AI处理完成, 但未生成文件。") # TODO:来个返回值
        return True, response_files, response_dict, messages
    else:
        print("未识别到有效文字。")
        return True, None, None, messages


def main():
    global global_count
    APPID = 'ec798696'
    APIKey = '079797c2b651aada7573f75eb2ca1955'
    APISecret = 'YzU5ZWM2NjJmOGY1ODQ0ZmM3M2ViZWEy'

    # TODO:让其回答一个1,加快速度
    initial_prompt = (
        "给我生成一份json文件，内容包括order_operation、table_id、snacks、drinks、other_items六个对象。"
        "order_operation包括0、1、2、3，分别代表ADD、DELETE、MODIFY、CHECK，"
        "执行一次送货的操作就是ADD；table_id包括TBD、1、2，其中左边的桌子赋值为1，右边的桌子赋值为2，TBD待定；"
        "snacks又包括需要搬运的不同的零食对象，其中1代表益达口香糖、2代表果蔬果冻、3代表C酷果冻、4代表伊利每益添乳酸菌、"
        "5代表金津陈皮丹，每种零食又有不同的值，代表有多少个；drinks又包括需要搬运的不同饮料对象，其中1代表咖啡，"
        "每种饮料又有不同的值，代表有多少个。other_items又包括不同的其他对象，从1开始上不设限，0则代表没有识别到其他对象，"
        "每种不同的对象有不同的值，代表有多少个。我的送货任务是：假如你是机器人，我对机器人说一段话，将一个C酷果冻放到左边的桌子上，"
        "然后你要识别出其中的关键信息，一个、C酷果冻、左边的桌子，并按照之前跟你描述的json对象格式一一填入，如果某个对象没有，都要一起返回，"
        "最终只返回一个完整的json内容，如果识别到的对象不在snacks和drinks的种类中，则填入other_items中，"
        "不要那么死板，如果识别到的关键物品的文字读音与例子中的读音类似,也可以认为是例子中的物品。"
        "例子中将一个C酷果冻放到左边的桌子上，则你只需要返回的文字内容是："
        "{\n"
        "    \"order_operation\": 0,\n"
        "    \"table_id\": 1,\n"
        "    \"snacks\": {\n"
        "        \"1\": 0,\n"
        "        \"2\": 0,\n"
        "        \"3\": 1,\n"
        "        \"4\": 0,\n"
        "        \"5\": 0\n"
        "    },\n"
        "    \"drinks\": {\n"
        "        \"1\": 0\n"
        "    },\n"
        "    \"other_items\": {\n"
        "        \"0\": 0\n"
        "    }\n"
        "}\n"
        "机器人一次只能运输同一张桌子上的东西，因此涉及到不同的桌子时要分成不同的json对象，涉及同一个桌子时可以合并成同一个json对象。"
        "不要生成其他任何无关json对象的文字内容，只需要生成上述的json内容,不需要任何有关生成内容的解释,我就是要你尽可能快的生成出json内容，那些多余的废话解释我一个也不需要！"
    )

    messages = [{"role": "user", "content": initial_prompt}]
    # 获取初始AI回答（不处理输出或保存）
    initial_response = chat_with_ai(messages)
    messages.append({"role": "assistant", "content": initial_response})
    if initial_response:
        print("系统初始化完成，等待语音识别...")

    # 列出当前连接的麦克风设备
    list_microphones()
    mic_index = get_valid_microphone_index()

    # 开始语音对话
    while True:
        try:
            continue_recognition, _, _, messages = voice_to_json(APPID, APIKey, APISecret, messages, duration=10,
                                                                 mic_index=mic_index)
            if not continue_recognition:
                break
        except KeyboardInterrupt:
            print("程序已终止。")
            break


if __name__ == "__main__":
    main()