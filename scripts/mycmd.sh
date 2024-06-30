###
 # @Author: JNU-ZH-C103 JNU_ZH_C103@163.com
 # @Date: 2024-06-02 16:26:08
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-06-10 12:18:49
 # @FilePath: /ros_ws/src/run_task/scripts/mycmd.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/
cd /home/zrt/xzc_code/Competition/AIRobot/ros_ws/
source ./devel/setup.bash
roslaunch run_task test_move_base.launch
rosrun tf view_frames

rosrun map_server map_saver -f /home/elephant/xzc_code/ros_ws/src/run_task/map/C111