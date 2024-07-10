from pymycobot import Mercury

ml = Mercury("/dev/left_arm")
mr = Mercury("/dev/right_arm")

ml.get_robot_status()