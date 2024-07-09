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

    cv2.imwrite(f'./cup_and_coffee/result/{timestamp}.jpg', image)
    
    xyz_list_with_id = []
    # 对于每个id都要进行位置检测
    with open(f'./cup_and_coffee/result/{timestamp}.txt', 'a') as file:
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
    parser.add_argument('--tag_size', type=float, default=27, help='The size of the tag in meters. Default is 20mm')
    
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

    base_coords        = [220.0, 200.0, 450.0, -90, 0.0, -90]
    
    for i in range(len(xyz_list)):
        id  = xyz_list[i][0]
        xyz = xyz_list[i][1]
        if id == 12:
            print(f"id {id} machie switch")
            print(f"base_coord {base_coords [:3]}")
            print(f"shousuan  xyz:{base_coords[0] + xyz[2] - 90 - 50 }  {base_coords[1] - xyz[1] - 78} {base_coords[2] + xyz[0] +40 -60}\r\n\r\n")

    
# 实际 [370.0, 134.0, 420.0, -90, 0.0, -90]
# 测量 408.3585694262373  133.79036186545983 476.8474648663423
    
    


    
    
    