from pymycobot import Mercury

ml = Mercury("/dev/left_arm")
mr = Mercury("/dev/right_arm")

# 上电
ml.power_on()
mr.power_on()

# 左右臂对齐

# 松开关节
mr.release_servo(1)
ml.release_servo(1)

# 锁紧关节
mr.focus_servo(1)
ml.focus_servo(1)

# 左臂清零
ml.set_servo_calibration(1)
ml.set_servo_calibration(2)
ml.set_servo_calibration(3)
ml.set_servo_calibration(4)
ml.set_servo_calibration(5)
ml.set_servo_calibration(6)

# 右臂清零
mr.set_servo_calibration(1)
mr.set_servo_calibration(2)
mr.set_servo_calibration(3)
mr.set_servo_calibration(4)
mr.set_servo_calibration(5)
mr.set_servo_calibration(6)


# 左右爪清零
# 抓爪通信打开
ml.set_gripper_mode(0)
mr.set_gripper_mode(0)

# 抓爪调零

ml.set_gripper_enabled(0)
mr.set_gripper_enabled(0)

ml.set_gripper_calibration()
mr.set_gripper_calibration()
