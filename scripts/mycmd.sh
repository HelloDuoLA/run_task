cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/
cd /home/zrt/xzc_code/Competition/AIRobot/ros_ws/
source ./devel/setup.bash
roslaunch run_task test_move_base.launch
rosrun tf view_frames

rosrun map_server map_saver -f /home/elephant/xzc_code/ros_ws/src/run_task/map/C111

git config --global credential.helper store  ·

# test