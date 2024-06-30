import cv2

# 2 bottom 摄像头 4 
camera_id = 6

camera = cv2.VideoCapture(camera_id)
# camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
i = 0

cv2.namedWindow('img', cv2.WINDOW_NORMAL)  # 创建一个可调整大小的窗口
cv2.resizeWindow('img', 1280, 720)  # 设置显示窗口的大小为1280x720

while 1:
    (grabbed, img) = camera.read()
    cv2.imshow('img',img)
    if cv2.waitKey(1) & 0xFF == ord('j'):  # 按j保存一张图片
        i += 1
        u = str(i)
        # firename=f'./01_left/{camera_id}_{u}.jpg'
        # firename=f'./01_right/{camera_id}_{u}.jpg'
        firename=f'./01_right/{camera_id}_{u}.jpg'
        # firename=f'./02_right/{camera_id}_{u}.jpg'
        firename=f'./STag/{camera_id}_{u}.jpg'
        cv2.imwrite(firename, img)
        print('写入：',firename)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


