from pymycobot import Mercury
import argparse
import time
import copy

# elephant02 左臂z轴向上偏
# 右臂 z轴需要+20, 且无法抓取下层陈皮丹

# elephant01 左臂完美
# 右臂 z轴不作处理


def grip(target_pose,arm:Mercury,arm_name:str, middle_pose):
    print(f"{arm_name} open gripper")
    ret = arm.set_gripper_state(0,100)
    time.sleep(2)
    print(f"{arm_name} open gripper. ret : {ret}")
    base_coords = arm.get_base_coords()
    print(f"{arm_name} base_coords : {base_coords}")
    change_z     = copy.deepcopy(base_coords)
    change_z[2]  = target_pose[2]
    
    change_y     = copy.deepcopy(change_z)
    change_y[1]  = target_pose[1]

    change_x     = copy.deepcopy(change_y)
    change_x [0] = target_pose[0]
    
    print(f"change_z        :{change_z}")
    print(f"change_y        :{change_y}")
    print(f"change_x        :{change_x }")
    print(f"{arm_name} base_coords  :{base_coords}")
    print("GOOOOOOOOOOOOOO!")
    # 改变高度
    print(f"{arm_name} changing hight. target {change_z}")
    ret = arm.send_base_coords(change_z,100)
    wait(arm)
    print(f"{arm_name} change hight ret : {ret}")
    
    # 改变y轴
    print(f"{arm_name} changing y. target {change_y}")
    ret = arm.send_base_coords(change_y,100)
    wait(arm)
    print(f"{arm_name} change y : {ret}")
    
    # 改变x轴
    print(f"{arm_name} changing x. target {change_x}")
    ret = arm.send_base_coords(change_x,100)
    wait(arm)
    print(f"{arm_name} change x : {ret}")
    
    print(f"{arm_name} close gripper")
    ret = arm.set_gripper_state(1,50)
    time.sleep(2)
    print(f"{arm_name} close gripper. ret : {ret}")
    
    print("put to container!!!!!!!!!!!!!!!!!")
    time.sleep(2)
    
    # 改变x
    change_x2 = copy.deepcopy(change_x)
    change_x2[0] = middle_pose[0]
    ret = arm.send_base_coords(change_x2,50)
    wait(arm)
    print(f"{arm_name} change x : {ret}")
    
    # 改变yz
    change_yz2 = copy.deepcopy(change_x2)
    change_yz2[1] = middle_pose[1]
    change_yz2[2] = middle_pose[2]
    ret = arm.send_base_coords(change_yz2,50)
    wait(arm)
    print(f"{arm_name} change yz2 : {ret}")
    
    # 改变角度
    change_angle = copy.deepcopy(change_yz2)
    change_angle[3] = middle_pose[3]
    change_angle[4] = middle_pose[4]
    change_angle[5] = middle_pose[5]
    ret = arm.send_base_coords(change_angle,50)
    wait(arm)
    print(f"{arm_name} change angle : {ret}")
    
    
    
def wait(arm:Mercury):
    time.sleep(0.3)
    while(arm.is_moving()):
        # print("arm is moving")
        time.sleep(0.03)



if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description="Control arm with position and side.")
    parser.add_argument("arm_id", choices=["left", "right"], help="Arm side: left or right")
    parser.add_argument("position", type=float, nargs=3, help="3 elements list for position: x, y, z")
    
    
    args = parser.parse_args()
    print(args.position)
    print(args.arm_id)
    
    ml = Mercury("/dev/left_arm")
    mr = Mercury("/dev/right_arm")
    
    power_result = ml.is_power_on()
    if power_result == None or power_result == False:
        power_result = ml.power_on()
        while power_result == None:
            time.sleep(0.3)
            power_result = ml.power_on()
    
    power_result = mr.is_power_on()
    if power_result == None or power_result == False:
        power_result = mr.power_on()

        while power_result == None:
            time.sleep(0.3)
            power_result = mr.power_on()

    ml.set_gripper_mode(0)
    mr.set_gripper_mode(0)
    
    left_middle_pose  = [230, 170 , 450, -180, 0, 0]
    right_middle_pose = [230, -170, 450,  180, 0, 0]

    if args.arm_id == "left":
        grip(args.position,ml,"left",left_middle_pose)
    else:
        grip(args.position,mr,"right",right_middle_pose)