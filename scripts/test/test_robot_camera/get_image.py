import cv2

# 2 bottom 摄像头
camera_id = 6

camera = cv2.VideoCapture(camera_id)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
i = 0
while 1:
    (grabbed, img) = camera.read()
    cv2.imshow('img',img)
    if cv2.waitKey(1) & 0xFF == ord('j'):  # 按j保存一张图片
        i += 1
        u = str(i)
        # firename=f'./01_left/{camera_id}_{u}.jpg'
        firename=f'./01_right/{camera_id}_{u}.jpg'
        cv2.imwrite(firename, img)
        print('写入：',firename)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


