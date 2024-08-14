import cv2
import time

# 初始化两个摄像头
camera_ids = [4, 6]
cameras = {cid: cv2.VideoCapture(cid, cv2.CAP_GSTREAMER) for cid in camera_ids}

# # 设置摄像头参数
for camera in cameras.values():
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

# camera_ids = [4, 6]
# cameras = {}

# for cid in camera_ids:
#     # 构建GStreamer管道字符串，包括帧率设置
#     gstreamer_pipeline = (
#         f'v4l2src device=/dev/video{cid} ! '
#         'video/x-raw, format=(string)YUY2, width=(int)640, height=(int)480, framerate=(fraction)30/1 ! '
#         'videoconvert ! appsink'
#     )
#     cameras[cid] = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

# 检查摄像头是否正确打开
for cid, camera in cameras.items():
    if not camera.isOpened():
        print(f"摄像头 {cid} 无法打开。")
    else:
        print(f"摄像头 {cid} 已成功打开。")
        

# 拍照次数
photo_count = 2

# 拍照函数
def take_photos(camera_id, prefix):
    camera = cameras[camera_id]
    for i in range(photo_count):
        grabbed, img = camera.read()
        if grabbed:
            timestamp = str(int(time.time()))
            firename = f'./STag/{timestamp}_{prefix}_{camera_id}_{i+1}.jpg'
            cv2.imwrite(firename, img)
            print('写入：', firename)
        time.sleep(1)  # 等待1秒拍下一张，确保时间戳不同

# 同时启动4、6号摄像头拍照
take_photos(6, 'left')
take_photos(4, 'right')

# 释放摄像头资源
for camera in cameras.values():
    camera.release()