from pymycobot import Mercury

ml = Mercury("/dev/left_arm")
mr = Mercury("/dev/right_arm")

mr.power_on()

mr.release_all_servos()

mr.focus_all_servos()

mr.release_servo(6)
mr.focus_servo(6)
mr.set_servo_calibration(1)
mr.get_angles()
mr.send_angle(1,0, 100)
mr.send_angle(2,30, 100)
mr.get_coords()
mr.get_base_coords()
# 0点[362.1, -541.4, 309.9, -90.0, 0.0, -59.99]
# [339.3, -243.1, 657.4, -125.86, -54.2, -29.61]
mr.send_base_coords([339.3, -243.1, 657.4, -125.86, -54.2, -29.61],50)

ml.get_base_coords()
# 0点[362.1, 541.4, 310.0, 89.99, 0.0, 59.99]
# [338.4, 241.4, 658.2, -126.07, -54.47, -89.34]
ml.send_base_coords([338.4, 241.4, 658.2, -126.07, -54.47, -89.34],50)


mr.send_angles([0,0,0,0,90,0],50)
# ml [90,50,0,0,90,0]

mr.send_angles([69.97, 44.71, -44.99, -90.0, 89.99, 30.0],50)
ml.send_angles([-70.0, 44.99, -45.00, 89.99, 89.99, 30.0],50)

# 机械臂出发点 [0,0,-90,0,90,0]
mr_al = [69.97, 44.71, -44.99, -90.0, 89.99, 30.0]
ml_al = [-70.0, 44.99, -45.00, 89.99, 89.99, 30.0]


mr.set_servo_calibration(1)
mr.set_servo_calibration(2)
mr.set_servo_calibration(3)
mr.set_servo_calibration(4)
mr.set_servo_calibration(5)
mr.set_servo_calibration(6)
# 左臂
from pymycobot import Mercury

ml = Mercury("/dev/left_arm")
ml.power_on()
ml.power_off()
ml.release_all_servos()
ml.focus_all_servos()

ml.release_servo(6)
ml.focus_servo(6)

ml.set_gripper_mode(0)
ml.get_gripper_mode()
ml.set_gripper_state(0,100)
ml.get_gripper_value()


ml.get_angles()
ml.set_servo_calibration(1)
ml.set_servo_calibration(2)
ml.set_servo_calibration(3)
ml.set_servo_calibration(4)
ml.set_servo_calibration(5)
ml.set_servo_calibration(6)

ml.send_angle(3,20, 100)
