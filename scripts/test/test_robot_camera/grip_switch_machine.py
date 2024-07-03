from pymycobot import Mercury
import argparse
import time
import copy



def grip_switch_machine(arm:Mercury,arm_name:str,target_pose):
    print("grip switch machine")
    base_coords = arm.get_base_coords()
    
    print(f"{arm_name} base_coords : {base_coords}")
    
    change_zy     = copy.deepcopy(base_coords)
    change_zy[1]  = target_pose[1]
    change_zy[2]  = target_pose[2]
    print(f"change_zy       :{change_zy}")

    change_x     = copy.deepcopy(change_zy)
    change_x [0] = target_pose[0]
    
    print(f"change_x       :{change_x }")

    print(f"{arm_name} base_coords  :{base_coords}")
    print("GOOOOOOOOOOOOOO!")
    # 改变高度
    print(f"{arm_name} changing zy. target {change_zy}")
    ret = arm.send_base_coords(change_zy,100)
    wait(arm)
    print(f"{arm_name} change zy ret : {ret}")
    
    # 改变x轴
    print(f"{arm_name} changing x. target {change_x}")
    ret = arm.send_base_coords(change_x,100)
    wait(arm)
    print(f"{arm_name} change x : {ret}")
    
    # 向上拨一下
    change_z3 = copy.deepcopy(change_x)
    change_z3[2] = change_z3[2] + 10
    ret = arm.send_base_coords(change_z3,100)
    wait(arm)
    print(f"{arm_name} change z3 : {ret}")
    
    

    print("close switch")
    time.sleep(2)
    
    change_z2    = copy.deepcopy(change_z3)
    change_z2[2] = change_z2[2] + 150
    ret = arm.send_base_coords(change_z2,100)
    wait(arm)
    print(f"change_z2       :{change_z2}")
    
    change_x2    = copy.deepcopy(change_z2)
    change_x2[0] = change_x2[0] + 50
    ret = arm.send_base_coords(change_x2,100)
    wait(arm)
    print(f"change_x2       :{change_x2}")
    
    change_z3    = copy.deepcopy(change_x2)
    change_z3[2] = change_z3[2] - 10 
    ret = arm.send_base_coords(change_z3,100)
    wait(arm)
    print(f"change_z3       :{change_z3}")
    
    
    print("return to origin point")
    ret = arm.send_base_coords(base_coords,100)
    wait(arm)
    
    
    
def wait(arm:Mercury):
    time.sleep(0.3)
    while(arm.is_moving()):
        # print("arm is moving")
        time.sleep(0.03)



if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description="Control arm with position and side.")
    # parser.add_argument("arm_id", choices=["left", "right"], help="Arm side: left or right")
    parser.add_argument("position", type=float, nargs=3, help="3 elements list for position: x, y, z")
    
    
    args = parser.parse_args()
    print(args.position)
    # print(args.arm_id)
    
    ml = Mercury("/dev/left_arm")
    # mr = Mercury("/dev/right_arm")

    ml.set_gripper_mode(0)
    # mr.set_gripper_mode(0)

    # if args.arm_id == "left":
    grip_switch_machine(ml,"left",args.position)
    # else:
    #     grip(args.position,mr,"right")