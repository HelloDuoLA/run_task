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
    
# 设置摄像头分辨率为1280x960
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

# 640 * 480
# cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)



# 创建保存图像的文件夹
folder1 = "/home/elephant/dev/team1/ros/src/run_task/data"
folder2 = "/home/elephant/dev/team1/ros/src/run_task/data"
os.makedirs(folder1, exist_ok=True)
os.makedirs(folder2, exist_ok=True)

try:
    while True:
        # 获取当前时间戳
        timestamp = int(time.time() * 1000)
        userInput = input("输入'save'保存图像，其他任意键继续: ")
        # 从第一个摄像头读取图像
        ret1, frame1 = cap1.read()
        # 从第二个摄像头读取图像
        ret2, frame2 = cap2.read()
        
        key = cv2.waitKey(1) & 0xFF
        if ret1 and ret2:
            # 等待按键事件，参数是等待时间（毫秒）
            # 如果设置为0，则无限等待直到按键
            
            # 获取当前时间戳
            timestamp = int(time.time() * 1000)
            # 构建文件名
            filename1 = os.path.join(folder1, f"{timestamp}_1.jpg")
            filename2 = os.path.join(folder2, f"{timestamp}_2.jpg")
            print(f"image1 name {filename1}")
            print(f"image2 name {filename2}")
            
            # 保存图像
            cv2.imwrite(filename1, frame1)
            cv2.imwrite(filename2, frame2)
        else:
            print("无法从摄像头获取图像")
            break
finally:
    # 释放资源
    cap1.release()
    cap2.release()