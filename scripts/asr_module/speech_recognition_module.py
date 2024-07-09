# speech_recognition_module.py

import websocket
import datetime
import hashlib
import base64
import hmac
import json
import time
import ssl
import wave
import os
import pyaudio
from wsgiref.handlers import format_date_time
import speech_recognition as sr
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
import _thread as thread
import logging

STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识

# 设置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(asctime)s - %(message)s')


class Ws_Param:
    # 初始化
    def __init__(self, APPID, APIKey, APISecret):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {"domain": "iat", "language": "zh_cn", "accent": "mandarin", "vinfo": 1, "vad_eos": 10000}

    # 生成url
    def create_url(self):
        url = 'wss://ws-api.xfyun.cn/v2/iat'
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        # 进行hmac-sha256进行加密
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        # 将请求的鉴权参数组合为字典
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        # 拼接鉴权参数，生成url
        url = url + '?' + urlencode(v)
        return url


class SpeechRecognizer:
    def __init__(self, APPID, APIKey, APISecret):
        self.wsParam = Ws_Param(APPID, APIKey, APISecret)
        self.ws = None
        self.recognized_text = ""
        self.is_recognizing = False

    def on_message(self, ws, message):
        try:
            code = json.loads(message)["code"]
            sid = json.loads(message)["sid"]
            if code != 0:
                errMsg = json.loads(message)["message"]
                logging.error(f"sid:{sid} call error:{errMsg} code is:{code}")
            else:
                data = json.loads(message)["data"]["result"]["ws"]
                result = ""
                for i in data:
                    for w in i["cw"]:
                        result += w["w"]
                logging.info(f"sid:{sid} call success!, data is: {json.dumps(data, ensure_ascii=False)}")
                self.recognized_text += result
        except Exception as e:
            logging.error(f"receive msg, but parse exception: {e}")

    def on_error(self, ws, error):
        logging.error(f"### error: {error}")

    def on_close(self, ws, a, b):
        logging.info("### closed ###")

    def on_open(self, ws):
        def run(*args):
            p = pyaudio.PyAudio()
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
            status = STATUS_FIRST_FRAME

            while self.is_recognizing:
                buf = stream.read(8000)
                if not buf:
                    status = STATUS_LAST_FRAME

                if status == STATUS_FIRST_FRAME:
                    d = {"common": self.wsParam.CommonArgs,
                         "business": self.wsParam.BusinessArgs,
                         "data": {"status": 0, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    d = json.dumps(d)
                    try:
                        ws.send(d)
                        logging.info("Sent first frame")
                    except websocket.WebSocketConnectionClosedException:
                        logging.error("WebSocket connection closed. Unable to send first frame.")
                        break
                    status = STATUS_CONTINUE_FRAME

                elif status == STATUS_CONTINUE_FRAME:
                    d = {"data": {"status": 1, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    try:
                        ws.send(json.dumps(d))
                        logging.info("Sent continue frame")
                    except websocket.WebSocketConnectionClosedException:
                        logging.error("WebSocket connection closed. Unable to send continue frame.")
                        break

                elif status == STATUS_LAST_FRAME:
                    d = {"data": {"status": 2, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    try:
                        ws.send(json.dumps(d))
                        logging.info("Sent last frame")
                    except websocket.WebSocketConnectionClosedException:
                        logging.error("WebSocket connection closed. Unable to send last frame.")
                    break
                time.sleep(0.04)

            stream.stop_stream()
            stream.close()
            p.terminate()
            ws.close()

        thread.start_new_thread(run, ())

    def start_recognition(self):
        self.recognized_text = ""
        self.is_recognizing = True
        wsUrl = self.wsParam.create_url()
        self.ws = websocket.WebSocketApp(wsUrl, on_message=self.on_message, on_error=self.on_error, on_close=self.on_close)
        self.ws.on_open = self.on_open
        self.ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    def stop_recognition(self):
        self.is_recognizing = False
        if self.ws:
            self.ws.close()
            self.ws = None
        return self.recognized_text

def save_audio_to_file(audio, base_dir, count):
    """
    将音频数据保存到文件中
    :param audio: 音频数据
    :param base_dir: 基础目录路径
    :param count: 第几次对话
    :return: 保存的文件路径
    """

    filename = f"order_{count}.wav"
    file_path = os.path.join(base_dir, filename)
    with open(file_path, "wb") as f:
        f.write(audio.get_wav_data())
    print(f"录音文件已保存：{file_path}")

    return file_path

def recognize_speech(APPID, APIKey, APISecret, count, start_timestamp, duration=10, mic_index=None):
    recognizer = sr.Recognizer()
    try:
        mic = sr.Microphone(device_index=mic_index)
    except IOError:
        print(f"无法初始化麦克风设备 {mic_index}，请检查设备连接并重试。")
        return "", None

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        print("请开始说话...")
        audio = None
        try:
            audio = recognizer.listen(source, timeout=duration, phrase_time_limit=duration)
            if audio:
                energy_threshold = recognizer.energy_threshold
                if audio.frame_data:
                    volume = sum(abs(sample) for sample in audio.frame_data) / len(audio.frame_data)
                    if volume > energy_threshold:
                        print("麦克风检测到声音。")
                    else:
                        print("麦克风未检测到足够的声音。")
                else:
                    print("未收到音频帧数据。")
        except sr.WaitTimeoutError:
            print("等待语音输入超时。")
            return ""

    print("语音识别结束，正在处理...")

    # 保存录音为 wav 文件
    base_dir = os.path.join("recordings", start_timestamp)
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    save_audio_to_file(audio, base_dir, count)

    try:
        recognized_text = recognizer.recognize_google(audio, language='zh-CN')
    except sr.UnknownValueError:
        recognized_text = ""
    except sr.RequestError as e:
        print(f"识别错误：{e}")
        recognized_text = ""

    return recognized_text