# 角度反关节, 1,4,6
# 坐标系反关节 2,4,6

# get_base_coords()

# !!!!!!!!!!!左臂
# 角度 90, 0, 90 爪子向前  
# 1. 向前 ml.send_angles([-43.3, -10.6, -126, 140, 110, -138],50)
# 1. 向前 ml.send_base_coords([345.7, 237.5, 452.7, 90.28, -0.2, 89.96],50)


# 向下观察食品框
# ml.send_base_coords([360, 170, 500, -180, 0.0, -90],50)
# ml.send_angles([-48.48, 26.74, -87.21, 65.36, 46.82, 26.16],50) 


# 2. 向前观察零食
# ml.send_base_coords([230, 170, 470, -90, 0.0, -90],50)
# ml.send_angles([-66.27, 5.44, -148.83, 123.8, 72.58, 24.6],50)


# 识别咖啡机
# ml.send_base_coords([220.0, 200.0, 450.0, -90, 0.0, -90],50)
# ml.send_angles([-71.36, -7.87, -155.8, 121.34, 73.92, 31.57],50)




# TODO:        ！!!!!!!!!!!!!!!!!右臂
# 1. 向前 mr.send_angles([ 43.5, -10.5, -126,-140, 110, 138],50)
# 1. 向前 mr.send_base_coords([346.1, -238.1, 453.3, -90.7, -0.26, -90.16],50)


# 1. 向下观察食品框
# mr.send_base_coords([360, -170, 500, -180, 0.0, 90],100)
# mr.send_angles([48.48, 26.74, -87.21, -65.36, 46.82, -26.16],50)

# 2. 向前观察零食
# mr.send_base_coords([230, -170, 470, 90, 0, 90],50)
# mr.send_angles([66.27, 5.44, -148.83, -123.8, 72.58, -24.6],50)

# 中转点1
# mr.send_base_coords([230, -170, 420, 90, 0, 90],100)
# mr.send_angles([57.2, -4.56, -161.46, -130.76, 73.97, -20.05],50)

# 中转点2
# mr.send_base_coords([360, -170, 420, -180, 0.0, 90],100)
# mr.send_angles([36.45, 12.08, -108.99, -69.12, 59.4, -23.14],50)

# 3. 识别杯子
# 直接从放容器的位置开始转会碰到机械臂
# mr.send_base_coords([50, -210, 200, 90, 0, 90],50)
# mr.send_angles([-135.79, 22.83, -152.09, 46.44, 56.43, 4.23],50)



# 3.1 放容器 -> 识别杯子的中间状态 : 
# 只能用角度,用坐标也会撞到机械臂
# mr.send_angles([0,0,-90,0,90,0],50)





