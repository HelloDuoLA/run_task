import pyaudio
import time
from speech_recognition_module import SpeechRecognizer
from ai_interaction_module import get_ai_response_as_dict
from speech_synthesis_module import tts
from config_loader import APPID, APIKey, APISecret

# 全局对话计数器
global_count = 0
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
    recognizer = SpeechRecognizer(APPID, APIKey, APISecret, mic_index)
    print("开始语音识别...")
    recognizer.start_recognition()
    recognizer.closed_event.wait()
    recognized_text = recognizer.stop_recognition()
    time.sleep(1)
    print(f"识别到的文字: {recognized_text}")

    if recognized_text:
        print("将识别的文字传递给AI处理...")
        global_count += 1  # 每次成功识别后递增计数
        messages.append({"role": "user", "content": recognized_text})
        response_files, response_dict, response = get_ai_response_as_dict(messages,
                                                count=global_count, start_timestamp=start_timestamp)
        messages.append({"role": "assistant", "content": response})
        if response_files:
            print("AI处理完成并生成了文件。")
        else:
            print("AI处理完成，但未生成文件。")
            # 调用语音合成函数
            tts(APPID, APIKey, APISecret, response)
        return True, response_files, response_dict, messages
    else:
        print("未识别到有效文字。")
        return True, None, None, messages


def main():
    global global_count
    use_default_microphone = True  # 设置为 True 时，直接使用默认麦克风设备

    messages = []  # 初始化空对话记录列表

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
                return False, None, None, messages
            continue_recognition, response_files, response_dict, messages = voice_to_json(APPID, APIKey, APISecret,
                                                        messages, mic_index)
            if response_files is None:
                print("识别到物品")
            else:
                print("未识别到物品")
            if not continue_recognition:
                break
        except KeyboardInterrupt:
            print("程序已终止。")
            break


if __name__ == "__main__":
    main()