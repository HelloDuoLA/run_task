import threading
import logging
import time
import hashlib
import base64
import hmac
import json
import ssl
import pyaudio
import websocket
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
import wave
import numpy as np
import os
from config_loader import silence_threshold, silence_duration, max_initial_wait

LOGDIR = "/home/elephant/dev/team1/ros/src/run_task/log/"

STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识

# 设置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')



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
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        url = url + '?' + urlencode(v)
        return url


class SpeechRecognizer:
    def __init__(self, APPID, APIKey, APISecret, global_count, mic_index=None):
        self.wsParam = Ws_Param(APPID, APIKey, APISecret)
        self.ws = None
        self.recognized_text = ""
        self.is_recognizing = False
        self.mic_index = mic_index
        self.silence_threshold = silence_threshold  # 从配置文件读取
        self.silence_duration = silence_duration    # 从配置文件读取
        self.max_initial_wait = max_initial_wait    # 从配置文件读取
        self.last_audio_time = 9999999999999999
        self.closed_event = threading.Event()  # 用于检测 WebSocket 关闭事件
        self.global_count = global_count

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
                if result.strip():  # 仅当结果不为空时记录日志
                    # print(f"sid:{sid} call success!, data is: {json.dumps(data, ensure_ascii=False)}")
                    print(f"sid:{sid} call success!")
                    self.recognized_text += result
                else:
                    print("recognize result is empty")
        except Exception as e:
            logging.error(f"receive msg, but parse exception: {e}")

    def on_error(self, ws, error):
        logging.error(f"### error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print("### closed ###")
        self.closed_event.set()  # 触发事件

    def on_open(self, ws):
        def run(*args):
            p = pyaudio.PyAudio()
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True,
                            # input_device_index=self.mic_index, frames_per_buffer=16000)
                            input_device_index=self.mic_index, frames_per_buffer=8000)
            status = STATUS_FIRST_FRAME

            frames = []
            initial_wait_start = time.time()
            self.last_audio_time = 9999999999999999 # 初始化最后音频时间

            while self.is_recognizing:
                # buf = stream.read(16000)
                buf = stream.read(8000)
                frames.append(buf)
                audio_data = np.frombuffer(buf, dtype=np.int16)

                # 检查是否静音
                audio_level = np.abs(audio_data).mean()
                print(f"Audio level: {audio_level}")  # 打印音频电平

                if audio_level > self.silence_threshold:
                    self.last_audio_time = time.time()
                    print("Detected speech, resetting last_audio_time")  # 检测到讲话，重置时间
                    
                # print(f"Global count: {self.global_count}") 
                # if self.global_count == 1:
                if (self.last_audio_time == 9999999999999999 and time.time() - initial_wait_start > self.max_initial_wait) \
                    or (self.last_audio_time != 9999999999999999 and time.time() - self.last_audio_time > self.silence_duration):
                    self.is_recognizing = False
                    status = STATUS_LAST_FRAME
                    print("Initial duration exceeded, stopping recognition")
                  
                if status == STATUS_LAST_FRAME:
                    break  # 立即退出循环

                if status == STATUS_FIRST_FRAME:
                    d = {"common": self.wsParam.CommonArgs,
                         "business": self.wsParam.BusinessArgs,
                         "data": {"status": 0, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    d = json.dumps(d)
                    try:
                        ws.send(d)
                        print("Sent first frame")
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
                        start_timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
                        print(f"{start_timestamp} Sent continue frame")
                    except websocket.WebSocketConnectionClosedException:
                        logging.error("WebSocket connection closed. Unable to send continue frame.")
                        break

                time.sleep(0.04)

            if status == STATUS_LAST_FRAME:
                d = {"data": {"status": 2, "format": "audio/L16;rate=16000",
                              "audio": str(base64.b64encode(buf), 'utf-8'),
                              "encoding": "raw"}}
                try:
                    ws.send(json.dumps(d))
                    print("Sent last frame")
                except websocket.WebSocketConnectionClosedException:
                    logging.error("WebSocket connection closed. Unable to send last frame.")

            stream.stop_stream()
            stream.close()
            p.terminate()
            ws.close()

            # 保存音频数据
            # threading.Thread(target=self.save_audio, args=(frames,)).start()

        threading.Thread(target=run).start()

    def save_audio(self, frames):
        base_dir = os.path.join( LOGDIR + "/recordings", time.strftime('%Y-%m-%d_%H-%M-%S'))
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)
        filename = f"order_{int(time.time())}.wav"
        file_path = os.path.join(base_dir, filename)

        with wave.open(file_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(pyaudio.PyAudio().get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b''.join(frames))
        print(f"录音文件已保存：{file_path}")

    def start_recognition(self):
        self.recognized_text = ""
        self.is_recognizing = True
        self.closed_event.clear()  # 清除事件
        wsUrl = self.wsParam.create_url()
        self.ws = websocket.WebSocketApp(wsUrl, on_message=self.on_message, on_error=self.on_error,
                                         on_close=self.on_close)
        self.ws.on_open = self.on_open
        self.ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    def stop_recognition(self):
        self.is_recognizing = False
        if self.ws:
            self.ws.close()
            self.ws = None
        # 异步处理后续任务
        threading.Thread(target=self.process_recognized_text).start()
        # self.process_recognized_text()
        return self.recognized_text

    def process_recognized_text(self):
        """
        加工recognized_text中的内容，确保所有的关键短语都替换为完整的短语。
        """
        # 定义替换规则，确保每个短语只替换一次
        replacements = {
            "拿": ["那", "哪"],
            "益达口香糖": ["口香糖", "益达", "亿达", "月达", "一达", "亿打","韵达","一挡"],
            # "果蔬果冻": ["古式果冻", "果素果冻"],
            # "C酷果冻": ["西裤果冻", "是酷果冻"],
            "伊利每益添乳酸菌": ["乳酸菌","5双均"],
            "金津陈皮丹": ["陈皮丹", "成绩单", "盛皮蛋", "成皮蛋", "层皮弹", "呈批单","沉迷单","成品单"],
            "咖啡": ["阿飞", "啊飞"]
        }

        # 进行替换，确保每个替换规则只被应用一次
        for full_phrase, keywords in replacements.items():
            for keyword in keywords:
                if keyword in self.recognized_text and full_phrase not in self.recognized_text:
                    self.recognized_text = self.recognized_text.replace(keyword, full_phrase)

        return self.recognized_text