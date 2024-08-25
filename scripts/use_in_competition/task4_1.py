from pymycobot import Mercury

ml = Mercury("/dev/left_arm")
mr = Mercury("/dev/right_arm")

# 上电
ml.power_on()
mr.power_on()

# 查看状态
ml.get_robot_status()
mr.get_robot_status()

# 关电
ml.power_off()
mr.power_off()

# 休闲位
# 1. 闲置动作
mr.send_base_coords([277, -145, 525, -180, 0.0, 90],100)
mr.send_angles([71.22, 62.77, -35.52, -66.98, 20.46, -35.6],50)

# 1. 休闲位
ml.send_base_coords([277, 145, 525, -180, 0.0, -90],50)
ml.send_angles([-71.22, 62.77, -35.52, 66.98, 20.46, 35.6],50) 

# 坐标运动前，使用关节运动到指定位置
ml.send_base_coords([277, 165, 525, -180,  0.0,-90],50)
mr.send_base_coords([277, -165, 525, -180, 0.0, 90],50)

# 查看坐标
ml.get_base_coords()
mr.get_base_coords()

# 坐标运动
ml.send_base_coords([])
mr.send_base_coords([])

# 抓取通信打开
ml.set_gripper_mode(0)
mr.set_gripper_mode(0)

# 获取通信
ml.get_gripper_mode()
mr.get_gripper_mode()

# 抓具打开
ml.set_gripper_state(0,50)
mr.set_gripper_state(0,50)

# 抓具关闭
ml.set_gripper_state(1,50)
mr.set_gripper_state(1,50)

