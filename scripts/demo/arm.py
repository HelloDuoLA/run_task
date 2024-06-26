'''
Description: 
Version: 1.0
Author: xzc
Date: 2024-06-03 12:50:30
'''
from pymycobot import Mercury
ml = Mercury('/dev/left_arm')
mr = Mercury('/dev/right_arm')
ml.power_on()
mr.power_on()
print(ml.get_angles())
print(mr.get_angles())