# 角度反关节, 1,4,6
# 坐标系反关节 2,4,6

# get_base_coords()

# 左臂
# 角度 90, 0, 90 爪子向前  
# 1. 向前 ml.send_angles([-43.3, -10.6, -126, 140, 110, -138],50)
# 1. 向前 ml.send_base_coords([345.7, 237.5, 452.7, 90.28, -0.2, 89.96],50)
# [348.3, 241.5, 569.6, 90.92, 0.29, 90.38]

# 2. 向前观察零食
# ml.send_base_coords([226.2, 179.5, 524.9, -90.86, 0.0, -89.69],50)
# ml.send_angles([-73.9, 13.99, -133.64, 118.89, 72.19, 32.37],50)

# 识别咖啡机
# ml.send_base_coords([262.5, 213.9, 701.2, -117.45, 0.91, -85.61],50)
# [-78.75, 67.94, -25.73, 121.15, 69.99, 80.97]

# 4. 夹零食盒 放在上方 
# mr.send_base_coords([376.3, 130, 550.4, 179.59, -1.1, 80.41],10)


# 5. 夹零食盒 下爪子  
# ml.send_base_coords([376.3, 130, 440.4, 179.59, -1.1, 80.41],50)

# 6. 夹起来速度慢一点

# 右臂
# 1. 向前 mr.send_angles([ 43.5, -10.5, -126,-140, 110, 138],50)
# 1. 向前 mr.send_base_coords([346.1, -238.1, 453.3, -90.7, -0.26, -90.16],50)


# 2. 向前观察零食
# mr.send_base_coords([230, -170, 525, 90, 0, 90],50)
# mr.send_angles([71.83, 15.82, -133.15, -120.72, 73.18, -30.76],50)


# 3. 识别杯子
# mr.send_base_coords([201.0, 0.2, 454.0, -147.63, 89.65, -147.77],50)
# mr.send_angles([49.79, 60.32, -127.82, -125.74, 54.86, -82.37],50)


# 4. 夹零食盒 放在上方 
# mr.send_base_coords([375.2, -130, 560.9, -178.99, -1.1, -80.21],10)


# 5. 夹零食盒 下爪子  
# mr.send_base_coords([375.2, -130, 460.9, -178.99, -1.1, -80.21],50)

# 6. 夹起来速度慢一点