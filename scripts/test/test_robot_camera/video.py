import cv2
 
 #创建摄像头捕获模块
cap = cv2.VideoCapture(0)
 
#创建窗口
window_handle = cv2.namedWindow("USB Camera", cv2.WINDOW_AUTOSIZE)
 
# 逐帧显示
while cv2.getWindowProperty("USB Camera", 0) >= 0:
    ret_val, img = cap.read()
    print(img.shape)
    
    # 图像太大需要调整
    height, width = img.shape[0:2]
    if width>800:
        new_width=800
        new_height=int(new_width/width*height)
        img = cv2.resize(img, (new_width,new_height))
 
    cv2.imshow("USB Camera", img)
 
    keyCode = cv2.waitKey(30) & 0xFF         
    if keyCode == 27:# ESC键退出
        break
 
#释放资源
cap.release()
cv2.destroyAllWindows()
