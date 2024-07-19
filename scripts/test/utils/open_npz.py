# -*- coding: utf-8 -*-
import numpy as np

camera_params = np.load("/home/zrt/xzc_code/Competition/AIRobot/ros_ws/src/run_task/other/camera_params.npz")  
mtx, dist = camera_params["mtx"], camera_params["dist"]
print(mtx)
print(dist)
