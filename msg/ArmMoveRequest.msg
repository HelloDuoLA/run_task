# 1.目标数据
uint16 task_index 
run_task/ArmPoseWithID arm_pose
uint8 arm_id # 0 - 左臂 1 - 右臂
uint8 arm_move_method # 移动方法
uint8 arm_speed       # 手臂移动速度
uint8 grasp_flag  # 0 - 打开 1 - 关闭 2. 不变
uint8 grasp_speed # 1 ~ 100 
bool  grasp_first # 爪子优先 0 - 不优先 1 - 优先