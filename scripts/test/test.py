from __future__ import annotations
import rospy
import os
import sys
import rospkg
import cv2
import numpy as np
import stag
import time
from tf.transformations import euler_matrix
from typing import List
import copy

rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import utilis
import run_task.msg as msg
import task
import order

STag_other_dict = {
    task.Task_image_rec.Rec_OBJ_type.CONTAINER        : 1,
    task.Task_image_rec.Rec_OBJ_type.MACHINE_SWITCH   : 2,
    task.Task_image_rec.Rec_OBJ_type.CUP              : 3,
    task.Task_image_rec.Rec_OBJ_type.WATER_POINT      : 4,
}

print("STag_other_dict", STag_other_dict)
print("task.Task_image_rec.Rec_OBJ_type.CONTAINER", STag_other_dict[task.Task_image_rec.Rec_OBJ_type.CONTAINER])