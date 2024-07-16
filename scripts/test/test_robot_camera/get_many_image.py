import cv2
import time
import os

# 摄像头索引，根据实际情况调整
camera_index1 = 4
camera_index2 = 6

# 创建VideoCapture对象
cap1 = cv2.VideoCapture(camera_index1)
cap2 = cv2.VideoCapture(camera_index2)

# 确保两个摄像头都成功打开
if not cap1.isOpened() or not cap2.isOpened():
    print("无法打开摄像头")
    exit()

# 创建保存图像的文件夹
folder1 = "/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/data"
folder2 = "/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/data"
os.makedirs(folder1, exist_ok=True)
os.makedirs(folder2, exist_ok=True)

try:
    while True:
        # 获取当前时间戳
        timestamp = int(time.time() * 1000)
        
        # 从第一个摄像头读取图像
        ret1, frame1 = cap1.read()
        # 从第二个摄像头读取图像
        ret2, frame2 = cap2.read()
        
        if ret1 and ret2:
            # 构建文件名
            filename1 = os.path.join(folder1, f"{timestamp}.jpg")
            filename2 = os.path.join(folder2, f"{timestamp}.jpg")
            
            # 保存图像
            cv2.imwrite(filename1, frame1)
            cv2.imwrite(filename2, frame2)
            
            # 等待1秒
            time.sleep(1)
        else:
            print("无法从摄像头获取图像")
            break
finally:
    # 释放资源
    cap1.release()
    cap2.release()