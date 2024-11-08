import websocket
import datetime
import hashlib
import base64
import hmac
import json
from urllib.parse import urlencode
import time
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread
import wave
import pyaudio
import io
from websocket import WebSocketApp

LOGDIR = "/home/jetson/code_ws/src/run_task/log/"

STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识


class Ws_Param(object):
    # 初始化
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.Text = Text

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "tte": "utf8"}
        self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-8')), "UTF8")}

    # 生成url
    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"
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

class SpeechSynthesizer:
    def __init__(self, APPID, APIKey, APISecret, text):
        self.wsParam = Ws_Param(APPID, APIKey, APISecret, text)
        self.ws = None
        self.audio_data = io.BytesIO()

    def on_message(self, ws, message):
        try:
            message = json.loads(message)
            code = message["code"]
            sid = message["sid"]
            audio = message["data"]["audio"]
            audio = base64.b64decode(audio)
            status = message["data"]["status"]

            if code != 0:
                errMsg = message["message"]
                print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
                ws.close()
            else:
                self.audio_data.write(audio)
                if status == 2:

                    ws.close()
                    time.sleep(1)  # 确保接收到所有数据后再进行下一步
                    self.audio_data.seek(0)
                    self.convert_pcm_to_wav()
                    self.play_audio()
        except Exception as e:
            print("receive msg,but parse exception:", e)

    def on_error(self, ws, error):
        print("### error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("")

    def on_open(self, ws):
        def run(*args):
            d = {"common": self.wsParam.CommonArgs,
                 "business": self.wsParam.BusinessArgs,
                 "data": self.wsParam.Data}
            d = json.dumps(d)

            ws.send(d)

        thread.start_new_thread(run, ())

    def convert_pcm_to_wav(self):
        self.audio_data.seek(0)
        pcm_data = self.audio_data.read()
        wav_data = io.BytesIO()
        with wave.open(wav_data, 'wb') as wf:
            wf.setnchannels(1)  # 单声道
            wf.setsampwidth(2)  # 16位采样
            wf.setframerate(16000)  # 采样率 16000 Hz
            wf.writeframes(pcm_data)
        self.audio_data = wav_data
        self.audio_data.seek(0)

    def play_audio(self):
        chunk = 1024
        wf = wave.open(self.audio_data, 'rb')
        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)
        data = wf.readframes(chunk)
        while data:
            stream.write(data)
            data = wf.readframes(chunk)
        stream.stop_stream()
        stream.close()
        p.terminate()

    def synthesize(self):
        websocket.enableTrace(False)
        wsUrl = self.wsParam.create_url()
        self.ws = WebSocketApp(wsUrl, on_message=self.on_message, on_error=self.on_error, on_close=self.on_close)
        self.ws.on_open = self.on_open
        self.ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

# 语音合成并播放音频
def tts(APPID, APIKey, APISecret, text):
    synthesizer = SpeechSynthesizer(APPID, APIKey, APISecret, text)
    synthesizer.synthesize()