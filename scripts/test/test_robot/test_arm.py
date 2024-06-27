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
