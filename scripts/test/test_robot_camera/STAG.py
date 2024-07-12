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
    
    timestamp = int(time.time())

    for i, id in enumerate(ids):
        print(f"Index: {i}, ID: {id[0]}")
        imagePoints = corners_list[i]
        print(f"imagePoints \n {imagePoints}")
        
        # 使用cv2.SOLVEPNP_IPPE_SQUARE
        success_square, rotationVector_square, translationVector_square = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        
        # 不使用cv2.SOLVEPNP_IPPE_SQUARE
        success, rotationVector, translationVector = cv2.solvePnP(objectPoints, imagePoints, mtx, distCoeffs)
        
        if success_square and success:
            print(f"使用cv2.SOLVEPNP_IPPE_SQUARE:")
            print(f"旋转向量 : {rotationVector_square[0][0]}, {rotationVector_square[1][0]}, {rotationVector_square[2][0]}")
            print(f"平移向量 x : {translationVector_square[0][0]}  y : {translationVector_square[1][0]} z : {translationVector_square[2][0]}")
            
            print(f"不使用cv2.SOLVEPNP_IPPE_SQUARE:")
            print(f"旋转向量 : {rotationVector[0][0]}, {rotationVector[1][0]}, {rotationVector[2][0]}")
            print(f"平移向量 x : {translationVector[0][0]}  y : {translationVector[1][0]} z : {translationVector[2][0]}")
            
            # 比较两种方法的结果差异
            rotation_diff = rotationVector_square - rotationVector
            translation_diff = translationVector_square - translationVector
            print(f"旋转向量差异 : {rotation_diff.ravel()}")
            print(f"平移向量差异 : {translation_diff.ravel()}")
        else:
            print(f"{i} {id} Failed to solve PnP")

            
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
        
    # print(f"Distortion Coefficients: {dist_coeffs}")
    
    image = cv2.imread(args.image_path)
    
    xyz_list = STag_rec(args.tag_size,camera_matrix,dist_coeffs,image,11)
    
# 左臂
# 测量 419.3141469292766  179.0925354305184 581.8427378387012


    
    
    