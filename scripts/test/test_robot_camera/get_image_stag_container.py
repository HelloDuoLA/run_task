import cv2
import time

# 初始化两个摄像头
camera_ids = [0, 2]
# cameras = {cid: cv2.VideoCapture(cid, cv2.CAP_GSTREAMER) for cid in camera_ids}

# def get_gs_pipeline(camera_id):
#     return f"v4l2src device=/dev/video{camera_id} io-mode=2 " \
#            f"! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 " \
#            f"! videoconvert " \
#            f"! video/x-raw, format=BGR " \
#            f"! appsink drop=1"

              

# cameras = {cid: cv2.VideoCapture(get_gs_pipeline(cid), cv2.CAP_GSTREAMER) for cid in camera_ids}
# cameras = {cid: cv2.VideoCapture(cid) for cid in camera_ids}
cameras = {cid: cv2.VideoCapture(cid,cv2.CAP_GSTREAMER) for cid in camera_ids}

for camera in cameras.values():
    if not camera.isOpened():
        print(f'摄像头 {camera} 打开失败')

# 设置摄像头参数
for camera in cameras.values():
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

# 拍照次数
photo_count = 2

# 拍照函数
def take_photos(camera_id, prefix):
    camera = cameras[camera_id]
    for i in range(photo_count):
        grabbed, img = camera.read()
        if grabbed:
            timestamp = str(int(time.time()))
            firename = f'./Image_stag/container/{timestamp}_{prefix}_{camera_id}_{i+1}.jpg'
            cv2.imwrite(firename, img)
            print('写入：', firename)
        time.sleep(1)  # 等待1秒拍下一张，确保时间戳不同

# 同时启动4、6号摄像头拍照
take_photos(camera_ids[0], 'left')
take_photos(camera_ids[1], 'right')

# 释放摄像头资源
for camera in cameras.values():
    camera.release()