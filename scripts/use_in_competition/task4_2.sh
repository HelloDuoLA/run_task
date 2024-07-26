# 初始化环境变量
source ./devel/setup.bash 

# 启动导航节点
roslaunch run_task mercury_navigation.launch 

# 启动键盘控制节点
roslaunch run_task mercury_keyboard_teleop.launch

