# 初始化环境变量
source ./devel/setup.bash 

# 启动建图节点
roslaunch run_task mercury_mapping.launch

# 启动键盘控制节点
roslaunch run_task mercury_keyboard_teleop.launch

# 使用的线速度和角速度为
# TODO:待测

# 保存地图
# TODO:待修改
rosrun map_server map_saver -f /home/elephant/xzc_code/ros_ws/src/run_task/map/C111