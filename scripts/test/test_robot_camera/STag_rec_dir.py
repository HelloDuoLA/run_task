import stag
import cv2
import argparse
import os
import glob

# 创建 ArgumentParser 对象
parser = argparse.ArgumentParser(description='Process images for STag marker detection.')
# 添加参数
parser.add_argument('image_folder', type=str, help='Path to the folder containing images')
# 解析命令行参数
args = parser.parse_args()

# 使用解析得到的参数
image_folder = args.image_folder
result_folder = os.path.join(image_folder, "result")

# 创建结果文件夹（如果不存在）
if not os.path.exists(result_folder):
    os.makedirs(result_folder)

# specify marker type
libraryHD = 11

# 获取文件夹中所有jpg文件
image_files = glob.glob(os.path.join(image_folder, "*.jpg"))

for image_path in image_files:
    # 加载图片
    image = cv2.imread(image_path)

    # 检测标记
    (corners, ids, rejected_corners) = stag.detectMarkers(image, libraryHD)
    print(f"Processing {os.path.basename(image_path)}: ids {ids}")
    
    # 绘制检测到的标记及其ID
    stag.drawDetectedMarkers(image, corners, ids)

    # 用不同颜色绘制未识别的四边形
    stag.drawDetectedMarkers(image, rejected_corners, border_color=(255, 0, 0))

    # 在结果文件夹中保存结果图片
    result_path = os.path.join(result_folder, os.path.basename(image_path))
    cv2.imwrite(result_path, image)