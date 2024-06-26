/*
 * @Description: 
 * @Version: 1.0
 * @Author: xzc
 * @Date: 2024-06-03 12:22:03
 */

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    setlocale(LC_ALL, "");
    //执行节点初始化
    ros::init(argc,argv,"HelloVSCode");

    //输出日志
    ROS_INFO("Hello VSCode!!!哈哈哈哈哈哈哈哈哈哈");
    return 0;
}
