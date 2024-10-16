cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/
cd /home/zrt/xzc_code/Competition/AIRobot/ros_ws/
source ./devel/setup.bash
roslaunch run_task test_move_base.launch
rosrun tf view_frames

rosrun map_server map_saver -f /home/elephant/xzc_code/ros_ws/src/run_task/map/C111

git config --global credential.helper store  ·

# 调参
rosrun rqt_reconfigure rqt_reconfigure

source ./devel/setup.bash 
roslaunch run_task mercury_mapping.launch 
roslaunch run_task mercury_navigation.launch 

git clone git@gitee.com:erzongxie/run_task.git --depth 1
git clone git@gitee.com:erzongxie/tensorrt_demos.git . --depth 1

git log --pretty=tformat: --numstat | awk '{ add += $1; subs += $2; loc += $1 - $2 } END { 
printf "added lines: %s, removed lines: %s, total lines: %s\n", add, subs, loc }'