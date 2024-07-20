from pymycobot import Mercury
import argparse
import time
import copy

def grip_cup(arm:Mercury,arm_name:str,cup_position:list,machine_position:list):
    print("griping cup")
    base_coords = arm.get_base_coords()
    print(f"{arm_name} get_base_coords : {base_coords}")
        

    
    # 改变xyz轴
    change_xyz1 = copy.deepcopy(base_coords)
    change_xyz1[0] = 250
    change_xyz1[1] = cup_position[1]
    change_xyz1[2] = cup_position[2]
    print(f"change_xyz1       :{change_xyz1}")
    ret = arm.send_base_coords(change_xyz1,100)
    wait(arm)
    
    # 打开爪子
    print(f"{arm_name} open gripper")
    ret = arm.set_gripper_state(0,100)
    time.sleep(2)
    print(f"{arm_name} open gripper. ret : {ret}")
    
    
    # 改变x轴
    change_x     = copy.deepcopy(change_xyz1)
    change_x [0] = cup_position[0]
    print(f"change_x        :{change_x }")
    
    print(f"{arm_name} changing x. target {change_x}")
    ret = arm.send_base_coords(change_x,100)
    wait(arm)
    print(f"{arm_name} change x : {ret}")
    
    print(f"{arm_name} close gripper")
    ret = arm.set_gripper_state(1,50)
    time.sleep(2)
    print(f"{arm_name} close gripper. ret : {ret}")
    
    print("water cup!!!!!!!!!!!!!")
    
    # 先移x轴
    machine_change_x    = copy.deepcopy(change_x)
    machine_change_x[0] = machine_position[0]
    ret = arm.send_base_coords(machine_change_x ,100)
    wait(arm)
    print(f"machine_change_x : {machine_change_x}")
    
    # yz轴一起移
    machine_change_yz    = copy.deepcopy(machine_change_x)
    machine_change_yz[1] = machine_position[1]
    machine_change_yz[2] = machine_position[2]
    print(f"machine_change_yz : {machine_change_yz}")
    ret = arm.send_base_coords(machine_change_yz ,100)
    wait(arm)
    
    
    print("back!!!!!!!!!!!!!!!!!")
    
    time.sleep(2)
    
    # 返回拿水杯后改变x轴的位置
    ret = arm.send_base_coords(machine_change_x ,100)
    wait(arm)
    print(f"return to machine_change_x : {machine_change_x}")
    
    # 返回拿水杯处
    ret = arm.send_base_coords(change_x ,100)
    wait(arm)
    print(f"return to change_x: {change_x}")
    
    # 松手
    print(f"{arm_name} open gripper")
    ret = arm.set_gripper_state(0,50)
    time.sleep(2)
    print(f"{arm_name} open gripper. ret : {ret}")
    
    # 返回原点
    print(f"{arm_name} moving to origin pose. target {base_coords}")
    ret = arm.send_base_coords(base_coords,100)
    wait(arm)
    print(f"{arm_name} moved to origin pose. ret : {ret}")
    
    # 闭爪
    print(f"{arm_name} close gripper")
    ret = arm.set_gripper_state(1,50)
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
    print(f"args.cup_position : {args.machine_position}")
    
    mr = Mercury("/dev/right_arm")
        
    power_result = mr.is_power_on()
    if power_result == None or power_result == False:
        power_result = mr.power_on()

        while power_result == None:
            time.sleep(0.3)
            power_result = mr.power_on()
            
    print(f"power_result : {power_result}")    
    
    # ml = Mercury("/dev/left_arm")


    # ml.set_gripper_mode(0)
    mr.set_gripper_mode(0)

    # if args.arm_id == "left":
    #     grip(args.position,ml,"left")
    # else:
    grip_cup(mr,"right",args.cup_position,args.machine_position)