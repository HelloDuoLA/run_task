#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import actionlib
import random
from pymycobot import Mercury

# 增加头文件路径 
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import run_task.msg as msg
import run_task.srv as srv
import robot


# 机械臂控制器
class arm_controller():
    def __init__(self,id:utilis.Device_id) -> None:
        self.id = id 
        if(self.id == utilis.Device_id.LEFT):
            self.action_name      = utilis.Topic_name.left_arm_action            # action名称
            self.pub_pose_topic   = utilis.Topic_name.left_arm_pose              # 发布机械臂状态
            self.control_instance = Mercury("/dev/left_arm")                     
        elif(self.id == utilis.Device_id.RIGHT):
            self.action_name      = utilis.Topic_name.right_arm_action           # action名称
            self.pub_pose_topic   = utilis.Topic_name.right_arm_pose             # 发布机械臂状态
            self.control_instance = Mercury("/dev/right_arm")                    # TODO需要修改 驱动句柄
        
        self.action = self.arm_action(self.action_name,self.control_instance,id)
        self.action.start_action()
    
    # 暂时不发布状态了
    # 发布机械臂状态
    # class arm_pose_pub():
    #     def __init__(self,topic_name,control_instance,id:utilis.Device_id) -> None:
    #         self.pub = rospy.Publisher(topic_name,msg.ArmPoseGrabPoseWithID,queue_size=10)
    #         self.control_instance = control_instance
    #         self.id = id
        
    #     # 发布
    #     def publish(self):
    #         arm_pose    = self.get_arm_pose()
    #         grab_status = self.get_grab_status()
    #         pub_pose    = msg.ArmPoseGrabPoseWithID()
    #         pub_pose.x  = arm_pose[0]
    #         pub_pose.y  = arm_pose[1]
    #         pub_pose.z  = arm_pose[2]
    #         pub_pose.rx = arm_pose[3]
    #         pub_pose.ry = arm_pose[4]
    #         pub_pose.rz = arm_pose[5]
    #         pub_pose.grab_status = grab_status
    #         pub_pose.id = self.id.value
    #         self.pub.publish(pub_pose)
            
    #     # 获取机械臂姿态
    #     def get_arm_pose(self):
    #         pass
        
    #     # 获取抓具状态
    #     def get_grab_status(self):
    #         pass
    
    
    # 机械臂 action 
    class arm_action():
        def __init__(self,action_name,control_instance) -> None:
            self.action_server = actionlib.SimpleActionServer(action_name, msg.MoveArmAction, self.execute_cb, False)
            self.control_instance = control_instance
            rospy.loginfo(f"node: {rospy.get_name()}, init {action_name} arm action server")
            
        # 启动action
        def start_action(self):    
            self.action_server.start()
        
        # TODO:根据实际机械臂Python API进行修改
        # 执行
        def execute_cb(self, goal:msg.MoveArmGoal):
            rospy.loginfo(f"node: {rospy.get_name()}, arm action server execute. goal: {goal}")
            # 1. 解析目标值
            goal_arm_pose     = utilis.Arm_pose(goal.arm_pose)
            goal_arm_pose_list= goal_arm_pose.to_list()
            goal_grasp_speed  = goal.grasp_speed
            goal_grasp_first  = goal.grasp_first
            goal_grasp_flag   = goal.grasp_flag
            change_grasp      = False # 是否改变夹具状态
            
            if goal_grasp_flag != robot.manipulation_status.clamp.status.DONTCANGE:
                change_grasp = True
            
            if change_grasp == True and goal_grasp_first == True:
                rospy.loginfo(f"node: {rospy.get_name()}, goal_grasp_flag: {goal_grasp_flag}")
                # 夹具动作
                if goal_grasp_flag == robot.manipulation_status.clamp.status.OPEN:
                    self.grab_release(goal_grasp_speed)       # 松开
                elif goal_grasp_flag == robot.manipulation_status.clamp.status.CLOSE:
                    self.grab_grip(goal_grasp_speed)          # 抓紧
            
            # finish = False           # 是否完成
            
            # 2. 给机械臂发送目标值
            self.control_instance.move_to(goal_arm_pose_list)
            
            # 3. 发送连续反馈
            rate = rospy.Rate(10) 
            while True:
                # 机械臂是否在运动
                if(self.is_arm_moving()):
                    # 在运动
                    feedback = msg.MoveArmFeedback()
                else:
                    # 不在运动
                    current_arm_pose = self.get_arm_status()
                    # 如果已经到位
                    if current_arm_pose == goal_arm_pose_list:
                        # finish = True
                        result = True
                        if change_grasp == True:
                            # 夹具动作
                            if goal_grasp_flag == robot.manipulation_status.clamp.status.OPEN:
                                self.grab_release(goal_grasp_speed)       # 松开
                            elif goal_grasp_flag == robot.manipulation_status.clamp.status.CLOSE:
                                self.grab_grip(goal_grasp_speed)          # 抓紧
                        break
                    else :
                        result = False
                        # finish = True
                        break
                
                rate.sleep()
                # 取消action
                if self.action_server.is_preempt_requested():
                    rospy.loginfo(f"node: {rospy.get_name()}, arm action server preempted")
                    self.action_server.set_aborted()
                    flag = False
                    break
            # 3. 响应最终结果

            rospy.loginfo(f"node: {rospy.get_name()}, arm action server finish. result: {result}")
            renturn_result = msg.MoveArmResult()
            if result:
                self.action_server.set_succeeded() #可以添加结果参数
            else:
                self.action_server.set_aborted()   #TODO: 失败是否是这样的
        
        # # 抓紧
        # def grab_grip(speed=0):
        #     pass
        
        # # 松开
        # def grab_release(speed=0):
        #     pass
        
        # # 获取抓具状态
        # def get_grab_status(self):
        #     pass
        
        # # 获取机械臂状态
        # def get_arm_status(self):
        #     pass
        
        # # 是否在运动
        # def is_arm_moving(self):
        #     pass
        
        # # 是否在目标位置
        # def is_at_goal(self):
        #     pass
        
        

def talker():
    # 初始化节点，命名为'talker'
    rospy.init_node('arm_node')

    # 设置发布消息的频率，1Hz
    rate = rospy.Rate(1)
    global left_arm_controller,right_arm_controller
    left_arm_controller  = arm_controller(utilis.Device_id.LEFT)
    right_arm_controller = arm_controller(utilis.Device_id.RIGHT)
    
    server = rospy.Service("CheckArmPose",srv.CheckArmPose,doCheckArmPose)
    
    while not rospy.is_shutdown():
        rospy.loginfo("arm_node")
        # 发布机械臂状态
        # left_arm_controller.arm_pose_pub.publish()
        # right_arm_controller.arm_pose_pub.publish()
        rate.sleep()

# 服务函数 
def doCheckArmPose(req):
    resp = srv.CheckArmPose()
    
    if req.arm_id == utilis.Device_id.LEFT:
        if req.type_id == 0:
            resp.pose = left_arm_controller.control_instance.get_base_coords()
        elif req.type_id == 1:
            resp.pose = left_arm_controller.control_instance.get_angles()
    elif req.arm_id == utilis.Device_id.RIGHT:
        if req.type_id == 0:
            resp.pose = right_arm_controller.control_instance.get_base_coords()
        elif req.type_id == 1:
            resp.pose = right_arm_controller.control_instance.get_angles()
    return resp

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
