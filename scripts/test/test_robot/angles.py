# 角度反关节, 1,4,6
# 坐标系反关节 2,4,6

# get_base_coords()

# 左臂
# 角度 90, 0, 90 爪子向前  
# 1. 向前 ml.send_angles([-43.3, -10.6, -126, 140, 110, -138],50)
# 1. 向前 ml.send_base_coords([345.7, 237.5, 452.7, 90.28, -0.2, 89.96],50)
# 389.13637645760053,280.36495036014435,405.00514013959497

# 2. 向前观察零食
# ml.send_base_coords([230, 170, 470, -90, 0.0, -90],50)
# ml.send_angles([-66.27, 5.44, -148.83, 123.8, 72.58, 24.6],50)
# ml.send_base_coords([425.1258045985535, 83.93756944273522, 470.6001210552953, -90.86, 0.0, -89.69],50)
# 
# ml.send_base_coords([367.3480529702553, 135.36657041997415, 666.5579732479903, -90.86, 0.0, -89.69],50)


# 识别咖啡机
# ml.send_base_coords([220.0, 200.0, 450.0, -90, 0.0, -90],50)
# ml.send_angles([-71.36, -7.87, -155.8, 121.34, 73.92, 31.57],50)


# 4. 夹零食盒 放在上方 
# mr.send_base_coords([376.3, 130, 550.4, 179.59, -1.1, 80.41],10)


# 5. 夹零食盒 下爪子  
# ml.send_base_coords([376.3, 130, 440.4, 179.59, -1.1, 80.41],50)

# 6. 夹起来速度慢一点

# ！!!!!!!!!!!!!!!!!右臂
# 1. 向前 mr.send_angles([ 43.5, -10.5, -126,-140, 110, 138],50)
# 1. 向前 mr.send_base_coords([346.1, -238.1, 453.3, -90.7, -0.26, -90.16],50)


# 2. 向前观察零食
# mr.send_base_coords([230, -170, 470, 90, 0, 90],50)
# mr.send_angles([66.27, 5.44, -148.83, -123.8, 72.58, -24.6],50)

# mr.send_base_coords([230, -170, 640, 90, 0, 90],50)
# mr.send_base_coords([452.02833857905307, -47.985909806908296, 460.9192967342742, 90, 0, 90],50)


# 3. 识别杯子
# mr.send_base_coords([50, -210, 210, 90, 0, 90],50)
# mr.send_angles([-138.4, 22.23, -154.04, 43.12, 57.25, 2.93],50)

# mr.send_base_coords([50, -210, 200, 90, 0, 90],50)
# mr.send_angles([-135.79, 22.83, -152.09, 46.43, 56.43, 4.24],50)
# xyz:400.49289308181454  -191.43492534874974 210

# 4. 夹零食盒 放在上方 
# mr.send_base_coords([380, -130, 460, -180, 0, -80.00],50)


# 5. 夹零食盒 下爪子  
# mr.send_base_coords([375.2, -130, 460.9, -178.99, -1.1, -80.21],50)

# 6. 夹起来速度慢一点