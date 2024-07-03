from pymycobot import Mercury
import argparse
import time
import copy

def grip_cup(target_pose,arm:Mercury,arm_name:str):
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
    
    print("back!!!!!!!!!!!!!!!!!")
    time.sleep(2)
    
    print(f"{arm_name} open gripper")
    ret = arm.set_gripper_state(0,100)
    time.sleep(2)
    print(f"{arm_name} open gripper. ret : {ret}")
    
    print(f"{arm_name} changing xy. target {change_z}")
    ret = arm.send_base_coords(change_z,100)
    wait(arm)
    print(f"{arm_name} change xy ret : {ret}")
    
    print(f"{arm_name} moving to origin pose. target {base_coords}")
    ret = arm.send_base_coords(base_coords,100)
    wait(arm)
    print(f"{arm_name} moved to origin pose. ret : {ret}")
    
    print(f"{arm_name} close gripper")
    ret = arm.set_gripper_state(1,100)
    time.sleep(2)
    print(f"{arm_name} open gripper. ret : {ret}")
        
    
    
    
    
def wait(arm:Mercury):
    time.sleep(0.3)
    while(arm.is_moving()):
        # print("arm is moving")
        time.sleep(0.03)



if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description="Control arm with position and side.")
    parser.add_argument("cup_position", type=float, nargs=3, help="3 elements list for cup position: x, y, z")
    parser.add_argument("machine_position", type=float, nargs=3, help="3 elements list for machine position: x, y, z")
    parser.add_argument("arm_id", choices=["left", "right"], default="left", help="Arm side: left or right", nargs='?')
    
    
    args = parser.parse_args()
    print(f"args.cup_position : {args.cup_position}")
    print(f"rgs.machine_position : {args.machine_position}")
    
    # ml = Mercury("/dev/left_arm")
    mr = Mercury("/dev/right_arm")

    # ml.set_gripper_mode(0)
    mr.set_gripper_mode(0)

    # if args.arm_id == "left":
    #     grip(args.position,ml,"left")
    # else:
    grip_cup(args.cup_position,args.machine_position)