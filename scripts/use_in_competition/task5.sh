# 建立地图

# 初始化环境变量
source ./devel/setup.bash 

# 启动建图节点
roslaunch run_task mercury_mapping.launch
# Rviz里可能没有订阅地图

# 启动键盘控制节点
roslaunch run_task mercury_keyboard_teleop.launch

# 使用的线速度和角速度为


# 角速度
- 0.2 # 左右 C按8下
# 线速度
- 0.8  # x按6下

# 保存地图
# TODO:待修改
rosrun map_server map_saver -f /home/elephant/mercury_x1_ros/src/turn_on_mercury_robot/map/map_team1_C111