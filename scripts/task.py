#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from enum import Enum,auto # 任务字典
import actionlib
from typing import List

# 自定义模块
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")
import utilis
import robot
import order
import arm


# 任务类型
class Task_type():
    # 功能任务
    class Task_function(Enum):
        PAUSE = 0                  # 暂停
    
    # 语音识别任务(弃用)
    class Task_speech_recognition(Enum):
        Speech_Recognition = 100   # 语音识别
    
    # 导航任务
    class Task_navigate(Enum):
        Navigate_to_destination    = 200             # 导航到目的地(任意目的点,还没确定点)
        Navigate_to_the_init_point = auto()          # 导航到起始点
        Navigate_to_the_snack_desk = auto()          # 导航到零食柜
        Navigate_to_the_drink_desk = auto()          # 导航到饮料柜
        Navigate_to_the_left_service_desk  = auto()  # 导航到左侧服务台
        Navigate_to_the_right_service_desk = auto()  # 导航到右侧服务台
        Navigate_to_one_service_desk       = auto()  # 导航到其中一个服务台
        Rotation_in_place                  = auto()  # 原地旋转
    
    # 图像识别任务
    class Task_image_rec(Enum):
        TBD           = 300          # 待定
        ALL_SNACK     = auto()     # 全部零食
        SNACK         = auto()     # 零食
        CONTIANER     = auto()     # 零食容器
        # COFFEE_MACHIE = auto()     # 咖啡机
        # CUP           = auto()     # 杯子
        CUP_COFFEE_MACHINE   = auto() # 杯子和咖啡机
        COFFEE_MACHIE_SWITCH = auto() # 咖啡机开关
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value,int):
                return self.value == value
    
    # 机械臂任务
    class Task_manipulation(Enum):
        Grasp              = 400                     # 夹具夹取
        Move               = auto()                  # 机械臂移动
        Grasp_snack        = auto()                  # 夹零食
        Lossen_snack       = auto()                  # 放零食
        Grasp_container    = auto()                  # 夹容器
        Deliever_container = auto()                  # 送容器
        Lossen_container   = auto()                  # 放容器
        Grasp_cup          = auto()                  # 夹杯子
        Deliever_cup       = auto()                  # 送杯子
        Lossen_cup         = auto()                  # 放杯子
        Turn_on_coffee_machine  = auto()             # 打开咖啡机
        Turn_off_coffee_machine = auto()             # 关闭咖啡机
    
    # 初始化
    def __init__(self,task_name:Enum) -> None:
        self.task_name = task_name                   # 任务名称
        self.task_type = task_name.__class__         # 任务类型
        
    def __str__(self) -> str:
        return f"{self.task_type.__name__} {self.task_name.name} "
    
    # 重写等式
    def __eq__(self, value: object) -> bool:
        return self.task_name == value

# 任务类
class Task():
    # latest_task_number   = 0                         # 已创建任务数量
    # latest_task_group_id = -1                        # 任务组ID
    # 任务状态
    class Task_status(Enum):
        NOTREADY    = 0                              # 未准备好
        BEREADY     = auto()                         # 准备好, 待开始
        RUNNING     = auto()                         # 运行中
        FINISHED    = auto()                         # 完成
    
        def __str__(self) -> str:
            return self.name
    
    # 任务结果
    class Task_result(Enum):
        FAILED = 0                                   # 失败
        SUCCEED = auto()                             # 成功
        INVALID = auto()                             # 无效
        
        def __str__(self) -> str:
            return self.name
    
    # 任务的可并行性
    class Task_parallel(Enum):
        NOTALLOWED = 0                               # 不允许并行
        # PREVIOUS   = auto()                          # 允许与前面的任务并行
        # NEXT       = auto()                          # 允许与后面的任务并行
        ALL        = auto()                          # 允许与前后的任务并行
        
        def __str__(self) -> str:
            return self.name
        
    # 初始化 
    def __init__(self,task_name,finish_cb):
        self.task_type   = Task_type(task_name)     # 任务信息 
        self.task_index  = -999                     # 任务序号 
        self.task_group_id = -999                   # 任务组ID
        self.create_time = rospy.Time.now()         # 创建时间
        self.start_time  = -999                     # 开始时间
        self.end_time    = -999                     # 结束时间
        self.finish_cb   = finish_cb                # 完成之后的回调函数
        self.status      = self.Task_status.BEREADY # 任务状态 
        self.task_result = self.Task_result.INVALID # 0: 失败 1: 成功
        self.return_data = None                     # 返回数据
        self.parallel    = self.Task_parallel.NOTALLOWED # 并行性, 默认不允许并行
        self.subtask_count = 1                      # 子任务数量,默认为1, 就是本任务
        self.subtask_count_finished = 0             # 已完成的子任务数量
        self.predecessor_tasks      = Task_sequence() # 相对前置任务
    
    # 添加前置任务
    def add_predecessor_task(self,task):
        self.predecessor_tasks.add(task)
        
    # 设置任务状态
    def set_status(self,status:Task_status):
        self.status = status
    
    # 获取任务状态
    def get_status(self):
        return self.status
    
    # 更新开始状态
    def update_start_status(self):
        self.start_time = rospy.Time.now()
        self.status = self.Task_status.RUNNING
    
    # 更新结束状态
    def update_end_status(self,result = Task_result.SUCCEED)->int:
        self.subtask_count_finished += 1
        
        # 如果完成, 更新状态与时间
        if self.if_finished():
            self.end_time = rospy.Time.now()
            self.status = self.Task_status.FINISHED
            # 如果子任务有失败,则任务失败
            if self.task_result != self.Task_result.FAILED:
                self.task_result = result

        return self.subtask_count - self.subtask_count_finished
    
    # 设置任务的组ID和序号
    def set_task_group_id_and_index(self,group_id,index):
        self.task_group_id = group_id
        self.task_index = index
    
    # 设置子任务数量
    def set_subtask_count(self,count):
        self.subtask_count = count
    
    # 判断该任务是否全部完成
    def if_finished(self):
        if self.subtask_count == self.subtask_count_finished:
            return True
        else:
            return False
    
    # 打印输出
    def __str__(self) -> str:
        rospy.loginfo(f"{rospy.get_name()} task id : {self.task_index}")
        rospy.loginfo(f"{rospy.get_name()} task name : {self.task_type}")
        predecessor_tasks_str = ""
        for task in self.predecessor_tasks.task_list:
            predecessor_tasks_str = predecessor_tasks_str + f"[{task.task_index}:{task.task_type}] "
        # return "OK"
        return (
                f"Task       : {self.task_type}\r\n"
                f"Group_id   : {self.task_group_id}\r\n" 
                f"Index      : {self.task_index}\r\n" 
                f"Create_time: {self.create_time}\r\n"
                f"Start_time : {self.start_time}\r\n"
                f"End_time   : {self.end_time}\r\n"
                f"Finsih_cb  : {self.finish_cb}\r\n"
                f"Status     : {self.status}\r\n"   
                f"Result     : {self.task_result}\r\n" 
                f"Return_data: {self.return_data}\r\n"
                f"Parallel   : {self.parallel}\r\n"
                f"Subtask_count: {self.subtask_count}\r\n"
                f"Subtask_count_finished: {self.subtask_count_finished}\r\n"
                f"Predecessor_tasks: {predecessor_tasks_str}\r\n"
                )
# 导航任务
class Task_navigation(Task):
    def __init__(self, task_name, finish_cb=None, target_3D_pose=utilis.Pose3D(), rotation_degree=0):
        super().__init__(task_name,finish_cb)
        self.target_3D_pose  = target_3D_pose
        self.rotation_degree = rotation_degree         # 旋转度数
    
    # 设置目的点
    def set_target(self,target_3D_pose:utilis.Pose3D):
        self.target_3D_pose = target_3D_pose
        
    # 打印输出
    def __str__(self) -> str:
        return super().__str__() + f"Target_2D_pose: {self.target_3D_pose} " + f"Rotation_degree: {self.rotation_degree}"

# 图像识别任务
class Task_image_rec(Task):
    def __init__(self, task_name,fn_callback, camera_id:utilis.Device_id, snack_list:order.Snack_list = order.Snack_list()):
        super().__init__(task_name,fn_callback)
        self.camera_id   = camera_id     # 摄像头id
        self.snack_list  = snack_list    # 零食列表
        self.need_modify_tasks = Task_sequence() # 需要修改的任务
    
    def set_snack_list(self,snack_list:order.Snack_list):
        self.snack_list = snack_list
    
    def add_need_modify_task(self,task):
        self.need_modify_tasks.add(task)
        
    def get_need_modify_task_count(self):
        return self.need_modify_tasks.get_task_count()
    
    def get_need_modify_task(self,index):
        return self.need_modify_tasks[index]
    
    # 打印输出
    def __str__(self) -> str:
        need_modify_tasks_str = "\r\n"
        for task in self.need_modify_tasks.task_list:
            need_modify_tasks_str = need_modify_tasks_str + 10 * " " + f"[{task.task_index}:{task.task_type}]\r\n"
        return super().__str__() + f"Camera_id: {self.camera_id} " + f"Snack_list: {self.snack_list}" + f"\r\nNeed_modify_tasks: {need_modify_tasks_str} "


# 机械臂运动、夹取任务
class Task_manipulation(Task):
    def __init__(self, task_name, fn_callback,arm_id:utilis.Device_id, target_arms_pose: List[arm.Arm_pose] = [arm.Arm_pose()],  \
                target_clamps_status: List[robot.manipulation_status.clamp.status] = [robot.manipulation_status.clamp.status.DONTCANGE,robot.manipulation_status.clamp.status.DONTCANGE], \
                clamp_speed = 50):
        super().__init__(task_name,fn_callback)
        self.arm_id              = arm_id               # 操作对象
        
        # 判断输入是不是列表
        if isinstance(target_arms_pose,arm.Arm_pose):
            self.target_arms_pose    = [target_arms_pose]
        else: # 是
            self.target_arms_pose    = target_arms_pose
        
        # 判断输入是不是列表
        if isinstance(target_clamps_status,robot.manipulation_status.clamp.status):
            self.target_clamps_status = [target_clamps_status]
        else:
            self.target_clamps_status = target_clamps_status
            
        self.clamp_speed         = clamp_speed          # 夹具速度
        self.clamp_first         = False                # 默认先动臂
    
    
    # 设置爪子先走
    def set_clamp_first(self):
        self.clamp_first = True
    
    # 设置臂先走
    def set_arm_first(self):
        self.clamp_first = False
    
    # 设置目标位置
    def set_target_arm_pose(self,arm_pose:arm.Arm_pose):
        self.target_arms_pose = arm_pose
        
    # 打印字符串
    def __str__(self) -> str:
        for i in range(len(self.target_arms_pose)):
            if i == 0:
                arms_pose_str = f"Arm_pose_{i}: {self.target_arms_pose[i]} "
            else:
                arms_pose_str = arms_pose_str + f"Arm_pose_{i}: {self.target_arms_pose[i]} "
        
        for i in range(len(self.target_clamps_status)):
            if i == 0:
                clamps_status_str = f"Clamp_status_{i}: {self.target_clamps_status[i]} "
            else:
                clamps_status_str = clamps_status_str + f"Clamp_status_{i}: {self.target_clamps_status[i]} "
        
        return super().__str__()+(
            f"Arm_id              : {self.arm_id} \r\n"
            f"Target_arms_pose    : {{{arms_pose_str}}} \r\n" 
            f"Target_clamps_status: {clamps_status_str} \r\n" 
            f"Clamp_speed         : {self.clamp_speed} \r\n"
            f"Clamp_first         : {self.clamp_first} \r\n"
        )

# 任务队列
class Task_sequence():
    def __init__(self) -> None:
        self.task_list : list[Task] = []
        
    # 打印输出
    def __str__(self) -> str:
        task_str = ""
        for task in self.task_list:
            task_str = task_str + f"\r\n\r\n{task}"
        return task_str
        # return "OK"
        # return
    
    # 添加任务
    def add(self,task):
        # 单任务
        if isinstance(task,Task):
            self.task_list.append(task)
        # 增加任务列表
        elif isinstance(task,Task_sequence):
            self.task_list.extend(task.task_list)
    
    # 更新组id和任务index
    def update_group_id(self,group_id):
        for index,task in enumerate(self.task_list):
            task.set_task_group_id_and_index(group_id,index)
    
    # 弹出第一个任务
    def pop(self):
        return self.task_list.pop(0)
    
    # 是否完成, 主要用于并行任务分析的前置任务
    def has_been_done(self):
        for task in self.task_list:
            # 如有一个任务未完成,则返回False
            if task.get_status() != Task.Task_status.FINISHED:
                return False
        return True

    def get_task_count(self):
        return len(self.task_list)

# 任务执行器中的任务管理器
class Task_manager_in_running():
    def __init__(self,parallel=True) -> None:
        self.task_dict = {}
        self.total_task_index = 0
        self._parallel = parallel

    def add_task(self,task:Task):
        if self._parallel:
            self.task_dict[self.total_task_index] = task
            self.total_task_index += 1
            return self.total_task_index - 1
        else:
            if len(self.task_dict) == 0:
                self.add_task(task)
                return 0
            else:
                # 抛出异常
                raise ValueError("Task is not allowed to parallel")
    
    def get_task(self,index=0)->Task:
        task = self.task_dict.pop(index)
        return task
    

