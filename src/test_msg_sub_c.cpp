#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <functional>

// 实际的处理函数，接收消息和额外的参数
void doPerson(const std_msgs::Int32::ConstPtr& msg, int extradata) {
    ROS_INFO("extra %d p %d", extradata, msg->data);
}

int main(int argc, char **argv) {
    // 1. 初始化ROS节点
    ros::init(argc, argv, "listener_person_p");
    ros::NodeHandle n;

    // 2. 创建订阅者对象
    int extradata = 66; // 额外的参数
    ros::Subscriber sub = n.subscribe<std_msgs::Int32>("test_msg", 10, 
        [extradata](const std_msgs::Int32::ConstPtr& msg) {
            doPerson(msg, extradata); // 使用lambda表达式捕获并传递额外的参数
        });

    // 3. 循环等待回调函数
    ros::spin();

    return 0;
}