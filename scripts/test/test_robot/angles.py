# 角度反关节, 1,4,6
# 坐标系反关节 2,4,6

# get_base_coords()

# !!!!!!!!!!!左臂
# 休闲位
# ml.send_base_coords([277, 145, 525, -180, 0.0, -90],50)
# ml.send_angles([-71.22, 62.77, -35.52, 66.98, 20.46, 35.6],50) 

# 向下观察食品框
# ml.send_base_coords([305, 255, 435, -180, 0.0, -90],50)
# ml.send_angles([-71.15, 20.44, -78.52, 65.38, 20.78, 33.89],50) 

# 2. 向前观察零食
# ! 和右臂不一样了
# ml.send_base_coords([160, 320, 360, -90, 0.0, -90],50)
# ml.send_angles([-119.88, -23.38, -141.87, 107.28, 51.84, 75.7],50)

# 2.5 向前拿零食中间点
# ml.send_base_coords([240, 200, 370, -90, 0, -90],50)
# ml.send_angles([-71.64, -12.0, -147.11, 123.6, 80.69, 42.74],80)

# 3. 放置零食中转点
# ml.send_base_coords([230, 250, 330, -90, 0, -90],50)
# ml.send_angles([-78.68, -36.32, -154.68, 121.5, 84.93, 60.03],80)


# 识别咖啡机 开机 
# ml.send_base_coords([160, 100, 330, -90, 0, -90],50)
# ml.send_angles([-80.2, 26.03, -155.94, 98.61, 59.67, -2.27],50)

# 识别咖啡机 关机
# ml.send_base_coords([160.0, 100, 450, -90, 0, -90],50)
# ml.send_angles([-84.06, 36.75, -124.11, 106.07, 63.69, 21.31],50)



# TODO:        ！!!!!!!!!!!!!!!!!右臂

# 1. 闲置动作
# mr.send_base_coords([277, -145, 525, -180, 0.0, 90],100)
# mr.send_angles([71.22, 62.77, -35.52, -66.98, 20.46, -35.6],50)

# 2. 向下观察食品框
# mr.send_base_coords([305, -48, 435, -180, 0.0, 90],100)
# mr.send_angles([53.66, 50.54, -74.79, -51.81, 48.91, -9.88],50)

# 3. 向前观察零食
# mr.send_base_coords([160, -90, 360, 90, 0, 90],50)
# mr.send_angles([80.08, 30.95, -147.32, -100.88, 60.31, -1.94],80)

# 放零食中转点
# mr.send_base_coords([230, -140, 330, 90, 0, 90],100)
# mr.send_angles([60.89, -2.3, -157.42, -128.06, 73.95, -22.48],100)


# 3. 识别杯子
# 直接从放容器的位置开始转会碰到机械臂
# mr.send_base_coords([20, -230, 70, 90, 0, 90],10)
# mr.send_angles([-145.1, 18.55, -139.3, 46.52, 43.04, 18.39],50)


# 抓杯子高度90就行了

# 3.1 放容器 -> 识别杯子的中间状态 : 
# 只能用角度,用坐标也会撞到机械臂
# mr.send_angles([0,0,-115,0,90,0],50)


# 4. 转移杯子
# mr.send_base_coords([300, 0, 140, 90, 0, 90],50)




