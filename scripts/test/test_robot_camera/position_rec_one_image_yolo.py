# -*- coding: utf-8 -*-
import argparse
import numpy as np
import cv2
import stag
import time
import numpy as np
from tf.transformations import euler_matrix

# 物体真实宽度
Obj_True_Width = {
    "guodong"  : 8 ,
    "yiliduo"  : 4.5 ,
    "chenpidan": 4.6 ,
    "yida"     : 5.6
}

# 物体真实高度
Obj_True_Height = {
    "guodong"  : 13.5 ,
    "yiliduo"  : 8.7  ,
    "chenpidan": 8.7  ,
    "yida"     : 8.3 
}
    


def STag_rec(tag_size,mtx,distCoeffs,image,libraryHD=11):
    # 假设的三维点 (例如，一个简单的正方形)
    objectPoints = np.array([
        [-tag_size/2, tag_size/2,  0],
        [tag_size/2 , tag_size/2,  0],
        [tag_size/2 , -tag_size/2 ,  0],
        [-tag_size/2, -tag_size/2 ,  0]
    ], dtype=np.float32)
    
    
    # 获取图像的尺寸
    height, width = image.shape[:2]

    # 计算图像中心作为圆心坐标
    center_coordinates = (width // 2, height // 2)
    radius = 5  # 圆的半径
    color = (0, 255, 0)  # BGR颜色，这里为绿色
    thickness = -1  # 设置为-1表示填充整个圆

    # 画圆
    cv2.circle(image, center_coordinates, radius, color, thickness)

    timestamp = int(time.time())

    cv2.imwrite(f'./STag/result/{timestamp}.jpg', image)
    
    xyz_list_with_id = []
    # 对于每个id都要进行位置检测
    with open(f'./YOLO/result/{timestamp}.txt', 'a') as file:
        for i, id in enumerate(ids):
            print(f"Index: {i}, ID: {id[0]}")
            file.write(f"Index: {i}, ID: {id[0]}\n")
            imagePoints  = corners_list[i]
            print(f"imagePoints \n {imagePoints}")
            # success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs)
            if success:
                xyz_list_with_id.append([id[0],translationVector.flatten().tolist()])
                # print(translationVector)
                # print(f"旋转向量 : {rotationVector[0][0]}, {rotationVector[1][0]}, {rotationVector[2][0]}")
                # print(f"平移向量 x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}")
                file.write(f"旋转向量 : {rotationVector[0][0]}, {rotationVector[1][0]}, {rotationVector[2][0]}\n")
                file.write(f"平移向量 x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}\n\n\n")
            else:
                print(f"{i} {id} Failed to solve PnP")
    
    return xyz_list_with_id

# 坐标转换
# ?
# yaw z轴
# roll x轴
# pitch y轴
def coords_trans(xyz, translation, rpy):
    roll  = np.radians(rpy[0])
    pitch = np.radians(rpy[1])
    yaw   = np.radians(rpy[2])
    rotation_matrix = euler_matrix(roll, pitch, yaw)[:3, :3]
    # print(f"rotation_matrix:\n {rotation_matrix}")
    new_xyz = np.dot(rotation_matrix, xyz) + translation

    return new_xyz
            
if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description='Process images for STag marker detection.')

    # 添加参数
    parser.add_argument('image_path', type=str, help='The path to the image file.')
    parser.add_argument('--calibration_path', type=str, default='./01_right', help='The path to the camera calibration folder. Default is ./calibration')
    parser.add_argument('--tag_size', type=float, default=24, help='The size of the tag in meters. Default is 20mm')
    
    # 解析参数
    args = parser.parse_args()
    
    # 假设calibration_path是你的标定文件夹路径
    calibration_path = args.calibration_path  # 使用argparse解析得到的路径
    mtx_file_path = f"{calibration_path}/mtx.txt"

    # 从文件读取数据
    with open(mtx_file_path, 'r') as file:
        lines = file.readlines()
        data = [list(map(float, line.split())) for line in lines]

    # 转换为numpy矩阵
    camera_matrix = np.array(data)
    # print(f"Camera Matrix: {camera_matrix}")
    
    dist_file_path = f"{calibration_path}/dist.txt"
    

    # 从文件读取畸变系数
    with open(dist_file_path, 'r') as file:
        line = file.readline()
        dist_coeffs = np.array(list(map(float, line.split())), dtype=np.float32)
        
        
    camera_matrix = np.array([[532.76634274 ,  0.,         306.22017641],
                    [  0.         , 532.94411762, 225.40097394],
                    [  0.         ,  0.         , 1.        ]], dtype=np.float32)
    
    dist_coeffs = np.array([ 0.17156132, -0.66222681, -0.00294354,  0.00106322,  0.73823942], dtype=np.float32)
        
    # print(f"Distortion Coefficients: {dist_coeffs}")
    
    image = cv2.imread(args.image_path)
    
    xyz_list = STag_rec(args.tag_size,camera_matrix,dist_coeffs,image,11)
   
    new_xyz_list = []
    base_coords_left        = [160, 250, 360, -90, 0.0, -90]
    base_coords_right       = [160, -140, 360, 90, 0, 90]
    yaw = -(-90)
    
    print(f"left   arm end pose {base_coords_left[:3]}")
    print(f"right  arm end pose {base_coords_right[:3]}")
    for i in range(len(xyz_list)):
        # print(f"id: {xyz_list[i][0]} xyz: {xyz_list[i][1]}")
        xyz = xyz_list[i][1]
        # 动态摄像头坐标系转 静态摄像头坐标系
        # static_image_coords = coords_trans(xyz, [0,0,0], [0,0,0])
        static_image_coords = coords_trans(xyz, [0,0,0], [0,0,-yaw])
        
        # 静态摄像头坐标系转手臂末端坐标系
        arm_end_coords = coords_trans(static_image_coords, [0,78,0], [0,0,0])
        
        # 手臂末端坐标系转机械臂坐标系
        arm_base_coords_left = coords_trans(arm_end_coords, base_coords_left[:3], base_coords_left[3:])
        arm_base_coords_right = coords_trans(arm_end_coords, base_coords_right[:3], base_coords_right[3:])
        
        print(f"id: {xyz_list[i][0]}")
        print(f"                xyz {xyz} ")
        print(f"static_image_coords {static_image_coords} ")
        print(f"arm_end_coords      {arm_end_coords} ")
        
        # new_xyz_list.append([xyz_list[i][0],arm_base_coords])
        # print(f"         newxyz: {new_xyz_list[i][1][0]},{new_xyz_list[i][1][1]},{new_xyz_list[i][1][2]}")
        print("left")
        print(f"left   arm end pose {base_coords_left[:3]}")
        # print(f"arm_base_coords {arm_base_coords_left} ")
        print(f"shousuan  x,y,z:{base_coords_left[0] + xyz[2] - 100} ,{base_coords_left[1] - xyz[1]-78 + 10}, {base_coords_left[2] + xyz[0] + 20}")
        print(f"shousuan    xyz:{base_coords_left[0] + xyz[2] - 100}  {base_coords_left[1] - xyz[1]-78 + 10} {base_coords_left[2] + xyz[0]  + 20}")
        print("right")
        print(f"right  arm end pose {base_coords_right[:3]}")
        # print(f"arm_base_coords {arm_base_coords_right} ")
        print(f"shousuan  x,y,z:{base_coords_right[0] + xyz[2] - 80 } , {base_coords_right[1] + xyz[1] + 78 -20}, {base_coords_right[2] - xyz[0] + 20}\r\n\r\n")
        print(f"shousuan    xyz:{base_coords_right[0] + xyz[2] - 80 }  {base_coords_right[1] + xyz[1] + 78 -20} {base_coords_right[2] - xyz[0]   + 20}\r\n\r\n")

    
# 左臂
# 测量 404.9347297360397  159.7166483771058 380.6525213499157
# 实际 394, 180, 380

# 上排 465
# 下排 260


# 右臂
# 测量 394, 180, 380
# 实际 
# 上排450
# 下排270




    
    
    