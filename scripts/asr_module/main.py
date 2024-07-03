import pyaudio
from speech_recognition_module import recognize_speech
from ai_interaction_module import get_ai_response_as_dict, chat_with_ai

# 全局对话计数器
global_count = 1


def list_microphones():
    """列出系统中连接的麦克风设备"""
    p = pyaudio.PyAudio()
    print("当前连接的麦克风设备：")
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info["maxInputChannels"] > 0:
            print(f"{i}: {device_info['name']}")
    p.terminate()


def prompt_user():
    """提示用户是否开始语音识别"""
    while True:
        choice = input("是否开始语音识别？ (y/n): ").strip().lower()
        if choice in ['y', 'n']:
            return choice == 'y'
        else:
            print("无效输入，请输入 'y' 或 'n'。")


def voice_to_json(APPID, APIKey, APISecret, messages, duration=20, mic_index=None):
    global global_count

    if not prompt_user():
        print("已取消本次语音识别。")
        return False, None, None, messages

    print("开始语音识别...")
    recognized_text = recognize_speech(APPID, APIKey, APISecret, duration, mic_index)
    print(f"识别到的文字: {recognized_text}")

    if recognized_text:
        messages.append({"role": "user", "content": recognized_text})
        response_files, response_dict, response = get_ai_response_as_dict(messages, count=global_count)
        messages.append({"role": "assistant", "content": response})
        if response_files:
            global_count += 1  # 每次成功识别并生成文件后递增计数
        return True, response_files, response_dict, messages
    else:
        print("未识别到有效文字。")
        return True, None, None, messages


def main():
    global global_count
    APPID = 'ec798696'
    APIKey = '079797c2b651aada7573f75eb2ca1955'
    APISecret = 'YzU5ZWM2NjJmOGY1ODQ0ZmM3M2ViZWEy'

    initial_prompt = (
        "给我生成一份json文件，内容包括order_operation、table_id、snacks、drinks四个对象。"
        "其中order_operation包括0、1、2、3，分别代表ADD、DELETE、MODIFY、CHECK，"
        "执行一次送货的操作就是ADD；table_id包括TBD、1、2，其中左边的桌子赋值为1，右边的桌子赋值为2，TBD待定；"
        "snacks又包括需要搬运的不同的零食对象，其中1代表益达口香糖、2代表果蔬果冻、3代表C酷果冻、4代表伊利每益添乳酸菌、"
        "5代表金津陈皮丹，每种零食又有不同的值，代表有多少个；drinks又包括需要搬运的不同饮料对象，其中1代表咖啡、2代表雪碧，"
        "每种饮料又有不同的值，代表有多少个。我的送货任务是：假如你是机器人，我对机器人说一段话，将一个C酷果冻放到左边的桌子上，"
        "然后你要识别出其中的关键信息，一个、C酷果冻、左边的桌子，并按照之前跟你描述的json对象格式一一填入，"
        "如果某个对象没有，都要一起返回，最终只返回一个完整的json内容，只输出json内容不需要再发送其他多余内容，"
        "如果其他零食没有提到，也要返回json对象，值定为0。"
    )

    messages = [{"role": "user", "content": initial_prompt}]
    # 获取初始AI回答（不处理输出或保存）
    initial_response = chat_with_ai(messages)
    messages.append({"role": "assistant", "content": initial_response})
    if initial_response:
        print("AI初始化响应已准备完毕。")

    # 列出当前连接的麦克风设备
    list_microphones()
    mic_index = int(input("请选择要使用的麦克风设备编号: "))

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
