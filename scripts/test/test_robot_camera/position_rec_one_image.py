import argparse
import numpy as np
import cv2
import stag
import time
import numpy as np
from tf.transformations import euler_matrix

def STag_rec(tag_size,mtx,distCoeffs,image,libraryHD=11):
    # 假设的三维点 (例如，一个简单的正方形)
    objectPoints = np.array([
        [-tag_size/2, -tag_size/2,  0],
        [tag_size/2 , -tag_size/2,  0],
        [tag_size/2 , tag_size/2 ,  0],
        [-tag_size/2, tag_size/2 ,  0]
    ], dtype=np.float32)
    
    print(objectPoints)
    
    (corners_list, ids, rejected_corners_list) = stag.detectMarkers(image, libraryHD)
    
    # draw detected markers with ids
    stag.drawDetectedMarkers(image, corners_list, ids)

    # draw rejected quads without ids with different color
    stag.drawDetectedMarkers(image, rejected_corners_list, border_color=(255, 0, 0))
    
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

    cv2.imwrite(f'./result/{timestamp}.jpg', image)
    
    xyz_list_with_id = []
    # 对于每个id都要进行位置检测
    with open(f'./result/{timestamp}.txt', 'a') as file:
        for i, id in enumerate(ids):
            print(f"Index: {i}, ID: {id[0]}")
            file.write(f"Index: {i}, ID: {id[0]}\n")
            imagePoints  = corners_list[i]
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
    parser.add_argument('--tag_size', type=float, default=20, help='The size of the tag in meters. Default is 20mm')
    
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
        
    # print(f"Distortion Coefficients: {dist_coeffs}")
    
    image = cv2.imread(args.image_path)
    
    xyz_list = STag_rec(args.tag_size,camera_matrix,dist_coeffs,image,11)
   
    # print(xyz_list) 
    # print(xyz_list[0][0])
    # print(xyz_list[0][1])
    new_xyz_list = []
    base_coords  = [226.2, 179.5, 524.9, -90.86, 0.0, -89.69]
    yaw = -(-90)
    
    print(f"arm end pose {base_coords[:3]}")
    for i in range(len(xyz_list)):
        # print(f"id: {xyz_list[i][0]} xyz: {xyz_list[i][1]}")
        xyz = xyz_list[i][1]
        # 动态摄像头坐标系转 静态摄像头坐标系
        # static_image_coords = coords_trans(xyz, [0,0,0], [0,0,0])
        static_image_coords = coords_trans(xyz, [0,0,0], [0,0,-yaw])
        

        
        # 静态摄像头坐标系转手臂末端坐标系
        arm_end_coords = coords_trans(static_image_coords, [0,78,0], [0,0,0])
        
        # 手臂末端坐标系转机械臂坐标系
        arm_base_coords = coords_trans(arm_end_coords, base_coords[:3], base_coords[3:])
        
        
        print(f"                xyz {xyz} ")
        print(f"static_image_coords {static_image_coords} ")
        print(f"arm_end_coords      {arm_end_coords} ")
        print(f"arm_base_coords     {arm_base_coords} ")
        print(f"shousuan    xyz:{base_coords[0] + xyz[2] - 90 }, {base_coords[1] - xyz[1]-78}, {base_coords[2] + xyz[0] +40}")
        print(f"shousuanooo xyz:{base_coords[0] + xyz[2]}, {base_coords[1] - xyz[1]}, {base_coords[2] + xyz[0]}")
         
        new_xyz_list.append([xyz_list[i][0],arm_base_coords])
        print(f"id: {xyz_list[i][0]} xyz: {new_xyz_list[i][1][0]},{new_xyz_list[i][1][1]},{new_xyz_list[i][1][2]}\n\n")
        
    
    
    
    


    
    
    