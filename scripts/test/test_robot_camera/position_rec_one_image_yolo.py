# -*- coding: utf-8 -*-
import argparse
import numpy as np
import cv2
import stag
import time
import numpy as np
from tf.transformations import euler_matrix
import sys
from enum import Enum,auto # 任务字典

# 将父目录添加到 sys.path
sys.path.append("../..") #相对路径或绝对路径
import engine


class Snack_id(Enum):
        TBD       = 0
        YIDA      = auto() #1
        GUODONG   = auto() #2
        RUSUANJUN = auto() #3
        CHENPIDAN = auto() #4
        


# 物体真实宽度
Obj_True_Width = {
    Snack_id.GUODONG.value    : 80 ,
    Snack_id.RUSUANJUN.value  : 45 ,
    Snack_id.CHENPIDAN.value  : 46 ,
    Snack_id.YIDA.value       : 56
}

# 物体真实高度
Obj_True_Height = {
    Snack_id.GUODONG.value    : 135 ,
    Snack_id.RUSUANJUN.value  : 87  ,
    Snack_id.CHENPIDAN.value  : 87  ,
    Snack_id.YIDA.value       : 83 
}

Tag_Snack_dict = {
        2   : Snack_id.GUODONG.value,
        1   : Snack_id.CHENPIDAN.value,
        3   : Snack_id.RUSUANJUN.value,
        -1  : Snack_id.YIDA.value,
}


def Yolo_rec(image, mtx, distCoeffs):
    model_name ="/home/elephant/dev/team1/model/ssd_resnet18_epoch_070.engine"
    INPUT_HW = (1280, 960)
    trt_ssd = engine.TrtSSD(model_name, INPUT_HW)
    # 识别
    boxes, confs, clss = trt_ssd.detect(image, 0.35)
    
    # 获取图像的尺寸
    height, width = image.shape[:2]

    # 计算图像中心作为圆心坐标
    center_coordinates = (width // 2, height // 2)
    radius = 5  # 圆的半径
    color = (0, 255, 0)  # BGR颜色，这里为绿色
    thickness = -1  # 设置为-1表示填充整个圆

    # 画圆
    cv2.circle(image, center_coordinates, radius, color, thickness)
    
    xyz_list_with_id = []
    for (box, conf, cls) in zip(boxes, confs, clss):
        print(f"conf {conf}")
        if conf < 0.9:
            print(f"conf < 0.9 {conf}. result discard")
            continue

            
        # 画框
        cv2.rectangle(image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)
            # 在边界框左上角添加一个绿色小圆点
        cv2.circle(image, (int(box[0]), int(box[1])), 5, (0, 255, 0), -1)

        # 在边界框右下角添加一个红色小圆点
        cv2.circle(image, (int(box[2]), int(box[3])), 5, (0, 0, 255), -1)

        # 在边界框旁边添加文本（类别和置信度）
        text = f'{cls}: {conf:.2f}'
        cv2.putText(image, text, (int(box[0]), int(box[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        obj_id = Tag_Snack_dict[cls]
        true_width  = Obj_True_Width[obj_id]
        true_height = Obj_True_Height[obj_id]
        
        

        objectPoints = np.array([
            [-true_height/2, -true_width/2,  0],
            [true_height/2 , -true_width/2,  0],
            [true_height/2 , true_width/2 ,  0],
            [-true_height/2, true_width/2 ,  0]
        ], dtype=np.float32) 
        

        
        print(f"true_width / true_height: {true_width / true_height}")
        print(f"cal_width / cal_height: { (box[3] - box[1]) / (box[2] - box[0])}")
        new_box_width = (box[2] - box[0]) * true_width / true_height
        print(f"error: {abs(new_box_width - (box[3] - box[1]))}")
        
        imagePoints = np.array([[
            [box[0], box[1]],
            [box[2], box[1]],
            [box[2], box[3]],
            [box[0], box[3]]
        ]], dtype=np.float32) 
        
        
        success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs)
        if success:
            xyz_list_with_id.append([cls,translationVector.flatten().tolist()])
            print(f"cls : {cls}  x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}")
        else:
            print(f"cls : {cls} Failed to solve PnP")
            
    # 保存图片
    timestamp = int(time.time())
    cv2.imwrite(f'./STag/result/{timestamp}.jpg', image)
    return xyz_list_with_id
            
if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description='Process images for STag marker detection.')

    # 添加参数
    parser.add_argument('image_path', type=str, help='The path to the image file.')
    parser.add_argument('--calibration_path', type=str, default='./01_right', help='The path to the camera calibration folder. Default is ./calibration')
    parser.add_argument('--tag_size', type=float, default=24, help='The size of the tag in meters. Default is 20mm')
    
    # 解析参数
    args = parser.parse_args()
        
        
    camera_matrix = np.array([[532.76634274 ,  0.,         306.22017641],
                                [  0.         , 532.94411762, 225.40097394],
                                [  0.         ,  0.         , 1.        ]], dtype=np.float32)
    
    dist_coeffs = np.array([ 0.17156132, -0.66222681, -0.00294354,  0.00106322,  0.73823942], dtype=np.float32)
    
    
    image = cv2.imread(args.image_path)
    
    xyz_list = Yolo_rec(image,camera_matrix,dist_coeffs)
    print(f"xyz_list :{xyz_list}")
    
    new_xyz_list = []
    base_coords_left        = [130, 250, 360, -90, 0, -90]
    base_coords_right       = [130, -160, 360, 90, 0, 90]
    
    print(f"left   arm end pose {base_coords_left[:3]}")
    print(f"right  arm end pose {base_coords_right[:3]}")
    for i in range(len(xyz_list)):
        xyz = xyz_list[i][1]
        # if (base_coords_right[2] < 350):
        #     continue
        print(f"\r\nid: {xyz_list[i][0]}")
        print(f"                xyz {xyz} ")
        
        print("left")
        print(f"left   arm end pose {base_coords_left[:3]}")
        # print(f"arm_base_coords {arm_base_coords_left} ")
        print(f"shousuan  x,y,z:{base_coords_left[0] + xyz[2] - 100} ,{base_coords_left[1] - xyz[1] - 78},    {base_coords_left[2] + xyz[0] + 20}")
        print(f"shousuan    xyz:{base_coords_left[0] + xyz[2] - 100}  {base_coords_left[1] - xyz[1] - 78}     {base_coords_left[2] + xyz[0]  + 20}")
        print("\r\nright")
        print(f"right  arm end pose {base_coords_right[:3]}")
        # print(f"arm_base_coords {arm_base_coords_right} ")
        print(f"shousuan  x,y,z:{base_coords_right[0] + xyz[2] - 80 } , {base_coords_right[1] + xyz[1] + 68}, {base_coords_right[2] - xyz[0] + 20}\r\n\r\n")
        print(f"shousuan    xyz:{base_coords_right[0] + xyz[2] - 80 }   {base_coords_right[1] + xyz[1] + 68}  {base_coords_right[2] - xyz[0] + 20}\r\n\r\n")

    
# 左臂
# 测量 404.9347297360397  159.7166483771058 380.6525213499157
# 实际 394, 180, 380

# 上排 430
# 下排 280


# 右臂
# 测量 394, 180, 380
# 实际 
# 上排430
# 下排280




    
    
    