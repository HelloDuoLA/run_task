# 角度反关节, 1,4,6
# 坐标系反关节 2,4,6

# get_base_coords()

# !!!!!!!!!!!左臂
# 角度 90, 0, 90 爪子向前  
# 1. 向前 ml.send_angles([-43.3, -10.6, -126, 140, 110, -138],50)
# 1. 向前 ml.send_base_coords([345.7, 237.5, 452.7, 90.28, -0.2, 89.96],50)


# 休闲位
# ml.send_base_coords([270, 170, 640, -180, 0.0, -90],50)
# ml.send_angles([-71.31, 63.55, -34.37, 67.69, 20.23, 36.37],50) 

# 向下观察食品框
# ml.send_base_coords([360, 220, 550, -180, 0.0, -90],50)
# ml.send_angles([-57.8, 32.65, -66.77, 75.38, 33.39, 42.65],50) 


# 2. 向前观察零食
# ! 和右臂不一样了
# ml.send_base_coords([180, 320, 470, -90, 0.0, -90],50)
# ml.send_angles([-108.42, -20.47, -140.94, 109.56, 60.68, 69.66],50)

# 2.5 向前拿零食中间点
# ml.send_base_coords([260, 200, 470, -90, 0, -90],50)
# ml.send_angles([-61.89, -4.64, -144.53, 129.69, 83.13, 34.92],80)


# 3. 放置零食中转点
# ml.send_base_coords([230, 170, 450, -180, 0, 0],50)
# ml.send_base_coords([230, 170 , 450, -90, 0, -90],50)


# 识别咖啡机 开机 
# ml.send_base_coords([220.0, 200.0, 450.0, -90, 0.0, -90],50)
# ml.send_angles([-71.36, -7.87, -155.8, 121.34, 73.92, 31.57],50)

# 识别咖啡机 关机
# ml.send_base_coords([220.0, 200.0, 550.0, -90, 0.0, -90],50)
# ml.send_angles([-78.94, 14.88, -126.71, 117.4, 73.22, 39.54],50)





# TODO:        ！!!!!!!!!!!!!!!!!右臂


# 1. 向前 mr.send_angles([ 43.5, -10.5, -126,-140, 110, 138],50)
# 1. 向前 mr.send_base_coords([346.1, -238.1, 453.3, -90.7, -0.26, -90.16],50)

# 1. 闲置动作
# mr.send_base_coords([270, -170, 640, -180, 0.0, 90],100)
# mr.send_angles([71.31, 63.55, -34.38, -67.73, 20.23, -36.42],50)

# 2. 向下观察食品框
# mr.send_base_coords([360, -150, 550, -180, 0.0, 90],100)
# mr.send_angles([52.75, 42.33, -66.09, -67.39, 40.94, -31.12],50)

# 3. 向前观察零食
# mr.send_base_coords([180, -140, 470, 90, 0, 90],50)
# mr.send_angles([76.87, 21.81, -150.56, -107.2, 61.99, -8.4],80)

# 放零食中转点
# mr.send_base_coords([230, -170, 450,  180, 0, 0],100)
# mr.send_angles([60.12, 9.55, -131.97, -36.26, 57.35, 98.41],100)

# YZ_XANGLE
# mr.send_base_coords([230, -170, 500,   90, 0, 90],100)


# 3. 识别杯子
# 直接从放容器的位置开始转会碰到机械臂
# mr.send_base_coords([20, -230, 190, 90, 0, 90],50)
# mr.send_angles([-141.77, 25.64, -142.14, 43.76, 50.75, 9.71],50)


# mr.send_base_coords([20, -250, 190, 90, 0, 90],50)
# mr.send_angles([-143.97, 21.38, -140.11, 44.98, 45.95, 14.87],50)

# mr.send_base_coords([20, -270, 190, 90, 0, 90],50)
# mr.send_angles([-146.21, 17.68, -137.97, 46.96, 41.21, 20.35],50)

# 3.1 放容器 -> 识别杯子的中间状态 : 
# 只能用角度,用坐标也会撞到机械臂
# mr.send_angles([0,0,-115,0,90,0],50)

# debug 备选
# mr.send_angles([0,0,-115,0,90,0],50)


# 4. 转移杯子
# mr.send_base_coords([300, 0, 240, 90, 0, 90],50)




