from pymycobot import Mercury
import argparse
import time
import copy

ml = Mercury("/dev/left_arm")
mr = Mercury("/dev/right_arm")

ml.set_gripper_mode(0)
mr.set_gripper_mode(0)

ml.set_gripper_state(0,100)



def grip(target_pose,arm_id):
    if arm_id == "left":
        print("ml open gripper")
        ret = ml.set_gripper_state(0,100)
        time.sleep(2)
        print(f"ml open gripper. ret : {ret}")
        base_coords = ml.get_base_coords()
        print(f"ml base_coords : {base_coords}")
        change_z     = copy.deepcopy(base_coords)
        change_z[2]  = target_pose[2]
        
        change_y     = copy.deepcopy(change_z)
        change_y[1]  = target_pose[1]

        change_x     = copy.deepcopy(change_y)
        change_x [0] = target_pose[0]
        
        print(f"change_z        :{change_z}")
        print(f"change_y        :{change_y}")
        print(f"change_x        :{change_x }")
        print(f"ml base_coords  :{base_coords}")
        print("GOOOOOOOOOOOOOO!")
        # 改变高度
        print(f"ml changing hight. target {change_z}")
        ret = ml.send_base_coords(change_z,100)
        wait(ml)
        print(f"ml change hight ret : {ret}")
        
        # 改变y轴
        print(f"ml changing y. target {change_y}")
        ret = ml.send_base_coords(change_y,100)
        wait(ml)
        print(f"ml change y : {ret}")
        
        # 改变x轴
        print(f"ml changing x. target {change_x}")
        ret = ml.send_base_coords(change_x,100)
        wait(ml)
        print(f"ml change x : {ret}")
        
        print(f"ml close gripper")
        ret = ml.set_gripper_state(1,50)
        time.sleep(2)
        print(f"ml close gripper. ret : {ret}")
        
        print("back!!!!!!!!!!!!!!!!!")
        time.sleep(2)
        
        print(f"ml open gripper")
        ret = ml.set_gripper_state(0,100)
        time.sleep(2)
        print(f"ml open gripper. ret : {ret}")
        
        print(f"ml changing xy. target {change_z}")
        ret = ml.send_base_coords(change_z,100)
        wait(ml)
        print(f"ml change xy ret : {ret}")
        
        print(f"ml moving to origin pose. target {base_coords}")
        ret = ml.send_base_coords(base_coords,100)
        wait(ml)
        print(f"ml moved to origin pose. ret : {ret}")
        
        print(f"ml close gripper")
        ret = ml.set_gripper_state(1,100)
        time.sleep(2)
        print(f"ml open gripper. ret : {ret}")
        
def wait(arm:Mercury):
    time.sleep(0.2)
    while(arm.is_moving()):
        print("arm is moving")
        time.sleep(0.03)



if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description="Control arm with position and side.")
    parser.add_argument("position", type=float, nargs=3, help="3 elements list for position: x, y, z")
    parser.add_argument("arm_id", choices=["left", "right"], help="Arm side: left or right")
    
    args = parser.parse_args()
    print(args.position)
    print(args.arm_id)
    grip(args.position,args.arm_id)