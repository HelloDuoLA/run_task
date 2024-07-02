from pymycobot import Mercury
import argparse
import time

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
        change_hight = base_coords
        change_hight[2]    = target_pose[2]
        target_pose_all    = base_coords
        target_pose_all[0] = target_pose[0]
        target_pose_all[1] = target_pose[1]
        target_pose_all[2] = target_pose[2]
        print(f"change_hight    :{change_hight}")
        print(f"target_pose_all :{target_pose_all}")
        print(f"ml base_coords : {base_coords}")
        
        # print(f"ml changing hight. target {change_hight}")
        # ret = ml.send_base_coords(change_hight,100)
        # print(f"ml change hight ret : {ret}")
        # print(f"ml moving to obj. target {target_pose}")
        # ret = ml.send_base_coords(target_pose,100)
        # print(f"ml moved to obj : {ret}")
        # print(f"ml close gripper")
        # ret = ml.set_gripper_state(1,50)
        # time.sleep(2)
        # print(f"ml close gripper. ret : {ret}")
        # time.sleep(2)
        
        # print(f"ml open gripper")
        # ret = ml.set_gripper_state(0,100)
        # time.sleep(2)
        # print(f"ml open gripper. ret : {ret}")
        
        # print(f"ml changing xy. target {change_hight}")
        # ret = ml.send_base_coords(change_hight,100)
        # print(f"ml change xy ret : {ret}")
        
        # print(f"ml moving to origin pose. target {base_coords}")
        # ret = ml.send_base_coords(base_coords,100)
        # print(f"ml moved to origin pose. ret : {ret}")
        
        # print(f"ml close gripper")
        # ret = ml.set_gripper_state(1,100)
        # time.sleep(2)
        # print(f"ml open gripper. ret : {ret}")
        
        


if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description="Control arm with position and side.")
    parser.add_argument("position", type=float, nargs=3, help="3 elements list for position: x, y, z")
    parser.add_argument("arm_id", choices=["left", "right"], help="Arm side: left or right")
    
    args = parser.parse_args()
    print(args.position)
    print(args.arm_id)
    grip(args.position,args.arm_id)