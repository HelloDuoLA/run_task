uint8 task_index   # 任务索引
uint8 task_type    # 请求任务标识: 1识别零食, 2识别食物框, 3识别咖啡机 4.识别杯子
uint8 camera_id    # 是否是右摄像头 3: 左或右 2:左和右 1:右 0:左
run_task/ArmPoseWithID[] arm_poses # 机械臂信息
run_task/SnackIDWithCount[] snacks # 零食信息