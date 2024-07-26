from pymycobot import Mercury

ml = Mercury("/dev/left_arm")
mr = Mercury("/dev/right_arm")

# 上电
ml.power_on()
mr.power_on()

# 关电
ml.power_off()
mr.power_off()

# 关节运动
mr.send_angles([0,0,0,0,90,0],50)
ml.send_angles([0,0,0,0,90,0],50)

# 坐标运动
ml.send_base_coords([])
mr.send_base_coords([])

# 抓取通信打开
ml.set_gripper_mode(0)
mr.set_gripper_mode(0)

# 抓具打开
ml.set_gripper_state(0,50)
mr.set_gripper_state(0,50)

# 抓具关闭
ml.set_gripper_state(1,50)
mr.set_gripper_state(1,50)

