from pymycobot import Mercury

ml = Mercury("/dev/left_arm")
mr = Mercury("/dev/right_arm")

ret = mr.power_on()
print(f"power on mr 1st. ret={ret}")
ret = mr.power_off()
print(f"power off mr. ret={ret}")


ret = ml.power_on()
print(f"power on ml 1st. ret={ret}")
ret = ml.power_off()
print(f"power off ml. ret={ret}")


ret = ml.power_on()
print(f"power on ml 2ed. ret={ret}")
ret = mr.power_on()
print(f"power on mr 2ed. ret={ret}")

ret = ml.get_angles()
print("ml.get_angles()= ", ret)

ret = mr.get_angles()
print("mr.get_angles()= ", ret)
