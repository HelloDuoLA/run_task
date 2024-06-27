import sys
from pymycobot import Mercury

# 检查命令行参数
if len(sys.argv) != 2 or sys.argv[1] not in ["left", "right"]:
    print("Usage: python all_servo_calibration.py <left|right>")
    sys.exit(1)

# 根据命令行参数选择机械臂
arm_port = "/dev/left_arm" if sys.argv[1] == "left" else "/dev/right_arm"
arm = Mercury(arm_port)
arm.power_on()

print(f"current angles : {arm.get_angles()}")
for i in range(1, 7):
    arm.set_servo_calibration(i)
print(f"current angles : {arm.get_angles()}")


