# import cv2
# camera=cv2.VideoCapture(0)
# i = 0
# while 1:
#     (grabbed, img) = camera.read()
#     cv2.imshow('img',img)
#     if cv2.waitKey(1) & 0xFF == ord('j'):  # 按j保存一张图片
#         i += 1
#         u = str(i)
#         firename=str('./img'+u+'.jpg')
#         cv2.imwrite(firename, img)
#         print('写入：',firename)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

import cv2

def count_cameras():
    index = 0
    count = 0
    while True:
        camera = cv2.VideoCapture(index)
        if not camera.isOpened():
            break
        count += 1
        camera.release()
        print(f"index : {index}")
        index += 1
    return count

num_cameras = count_cameras()
print(f"系统中有 {num_cameras} 个摄像头。")