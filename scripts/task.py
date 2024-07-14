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
import log


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
        Navigate_to_middle_point   = auto()          # 导航到中间点
        Navigate_to_the_drink_desk = auto()          # 导航到饮料柜
        Navigate_to_the_left_service_desk  = auto()  # 导航到左侧服务台
        Navigate_to_the_right_service_desk = auto()  # 导航到右侧服务台
        Navigate_to_one_service_desk       = auto()  # 导航到其中一个服务台
        Rotation_in_place                  = auto()  # 原地旋转
        Move_backward                      = auto()  # 后退
        Move_forward                       = auto()  # 前进
        
        
        def __eq__(self, value: object) -> bool:
            try:
                # 尝试比较对方的value属性
                return self.value == value.value
            except AttributeError:
                # 如果对方没有value属性，比较对方本身
                return self.value == value

        
    
    # 图像识别任务
    class Task_image_rec(Enum):
        TBD           = 300           # 待定
        ALL_SNACK     = auto()        # 全部零食   301
        SNACK         = auto()        # 零食       302
        CONTAINER     = auto()        # 零食容器   303
        CUP_COFFEE_MACHINE        = auto() # 杯子和咖啡机
        COFFEE_MACHINE_SWITCH_ON  = auto() # 咖啡机开关, 用于开机
        COFFEE_MACHINE_SWITCH_OFF = auto() # 咖啡机开关, 用于关机
        
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value,int):
                return self.value == value
    
    # 机械臂任务
    class Task_manipulation(Enum):
        Grasp              = 400                     # 夹具夹取
        Move               = auto()                  # 机械臂移动
        Move_to_IDLE       = auto()                  # 机械臂空闲
        Grasp_snack        = auto()                  # 夹零食
        Lossen_snack       = auto()                  # 放零食
        Rec_snack          = auto()                  # 识别零食
        Grasp_container    = auto()                  # 夹容器
        Deliever_container = auto()                  # 送容器
        Rec_container      = auto()                  # 识别容器
        Lossen_container   = auto()                  # 放容器
        Grasp_cup          = auto()                  # 夹杯子
        Grasp_cup_pre      = auto()                  # 夹杯子 准备
        Water_cup          = auto()                  # 杯子接水
        Deliever_cup       = auto()                  # 送杯子
        Lossen_cup         = auto()                  # 放杯子
        Rec_cup_machine    = auto()                  # 识别杯子和咖啡机
        Turn_on_coffee_machine  = auto()             # 打开咖啡机
        Turn_off_coffee_machine = auto()             # 关闭咖啡机
        Rec_machine_switch = auto()                  # 识别咖啡机开关
    
    # 初始化
    def __init__(self,task_name:Task_manipulation) -> None:
        self.task_name = task_name                   # 任务名称
        self.task_type = task_name                   # 任务类型
        
    def __str__(self) -> str:
        # print(f"self.task_type : {self.task_type}")
        return f"{self.task_type.__class__.__name__} {self.task_type.name}"
    
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
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value,int):
                return self.value == value
    
    # 任务结果
    class Task_result(Enum):
        FAILED = 0                                   # 失败
        SUCCEED = auto()                             # 成功
        INVALID = auto()                             # 无效
        
        def __str__(self) -> str:
            return self.name
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value,int):
                return self.value == value
    
    # 任务的可并行性
    class Task_parallel(Enum):
        NOTALLOWED = 0                               # 不允许并行
        # PREVIOUS   = auto()                          # 允许与前面的任务并行
        # NEXT       = auto()                          # 允许与后面的任务并行
        ALL        = auto()                          # 允许与前后的任务并行
        
        def __str__(self) -> str:
            return self.name
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value,int):
                return self.value == value
        
    # 初始化 
    def __init__(self,task_name,finish_cb,name=""):
        self.task_type   = Task_type(task_name)     # 任务信息 
        self.task_index  = 0                        # 任务序号 
        self.task_group_id = -999                   # 任务组ID
        self.create_time = rospy.Time.now()         # 创建时间
        self.start_time  = -999                     # 开始时间
        self.end_time    = -999                     # 结束时间
        self.finish_cb   = None                # 完成之后的回调函数
        self.status      = self.Task_status.BEREADY # 任务状态 
        self.task_result = self.Task_result.INVALID # 0: 失败 1: 成功
        self.return_data = None                     # 返回数据
        self.parallel    = self.Task_parallel.NOTALLOWED # 并行性, 默认不允许并行
        self.subtask_count = 1                      # 子任务数量,默认为1, 就是本任务
        self.subtask_count_finished = 0             # 已完成的子任务数量
        self.predecessor_tasks      = Task_sequence() # 相对前置任务
        self.name = name                            # 任务名称
        self.sleep_time_after_task  = 0             # 任务完成休眠时间
        self.sleep_time_before_task = 0.1           # 开始任务前休眠时间

    
    # 设置任务完成休眠时间
    def set_sleep_time_after_task(self,sleep_time_after_task_second):
        self.sleep_time_after_task = sleep_time_after_task_second
        
    # 设置开始任务前休眠时间
    def set_sleep_time_before_task(self,sleep_time_before_task):
        self.sleep_time_before_task = sleep_time_before_task
    
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
    
    # 判断任务是否相等
    def __eq__(self, value: object) -> bool:
        if self.task_group_id == value.task_group_id and self.task_index == value.task_index:
            return True
        else:
            return False
    
    # 打印输出
    def __str__(self) -> str:
        # rospy.loginfo(f"{rospy.get_name()} task id : {self.task_index}")
        # rospy.loginfo(f"{rospy.get_name()} task name : {self.task_type}")
        predecessor_tasks_str = ""
        for task in self.predecessor_tasks.task_list:
            predecessor_tasks_str = predecessor_tasks_str + f"[{task.task_index}:{task.task_type}] "
        # return "OK"
        return (
                f"Task       : {self.task_type}\r\n"
                f"Name       : {self.name}\r\n"
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
                f"Sleep_time_before_task : { self.sleep_time_before_task }"
                f"Sleep_time_after_task  : { self.sleep_time_after_task  }"
                )
        
# 功能任务
class Task_function(Task):
    def __init__(self, task_name:Task_type.Task_function, finish_cb=None,name=""):
        super().__init__(task_name,finish_cb,name)
    
    # 打印输出
    def __str__(self) -> str:
        return super().__str__() 

        
# 导航任务
class Task_navigation(Task):
    def __init__(self, task_name, finish_cb=None, target_3D_pose=utilis.Pose3D(),move_back_speed=0.15,name=""):
        super().__init__(task_name,finish_cb,name)
        self.target_3D_pose  = target_3D_pose
        self.move_back_second  = 4                  # 后退时间
    
    # 设置目的点
    def set_target(self,target_3D_pose:utilis.Pose3D):
        self.target_3D_pose = target_3D_pose
    
    # 设置后退时间 
    def set_move_back_second(self,second):
        self.move_back_second = second
        
    # 打印输出
    def __str__(self) -> str:
        if self.name != Task_type.Task_navigate.Move_backward:
            return super().__str__() + f"Target_3D_pose: {self.target_3D_pose} "
        else:
            return super().__str__() +  f"back_meters: x {self.target_3D_pose.x}, y {self.target_3D_pose.y}"

# 图像识别任务
class Task_image_rec(Task):
    class Rec_OBJ_type(Enum):
        LOSSEN_SNACK   = 0        # 松开零食
        CONTAINER      = auto()   # 容器
        MACHINE_SWITCH = auto()   # 机器开关
        CUP            = auto()   # 杯子
        WATER_POINT    = auto()   # 接水点
        
        def __eq__(self, value: object) -> bool:
            if isinstance(value,self.__class__):
                return self.value == value.value
            elif isinstance(value,int):
                return self.value == value
        
        # 但要确保同时定义一个 __hash__ 方法，以保持枚举成员的可哈希性
        def __hash__(self):
            return hash(self.value)
        
    
    def __init__(self, task_name,fn_callback, camera_id:utilis.Device_id, snack_list:order.Snack_list = order.Snack_list(),name=""):
        super().__init__(task_name,fn_callback,name)
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
        return super().__str__() + f"Camera_id: {self.camera_id} \n" + f"Snack_list: {self.snack_list}" + f"\nNeed_modify_tasks: {need_modify_tasks_str} "


# 机械臂运动、夹取任务
class Task_manipulation(Task):
    def __init__(self, task_name, fn_callback,arm_id:utilis.Device_id, target_arms_pose: List[arm.Arm_pose] = [arm.Arm_pose()],  \
                target_clamps_status: List[arm.GripMethod] = [arm.GripMethod.DONTCANGE,arm.GripMethod.DONTCANGE], \
                clamp_speed = 50, arm_move_method = arm.ArmMoveMethod.XYZ, click_length = 0,name=""):
        super().__init__(task_name,fn_callback,name)
        self.arm_id              = arm_id               # 操作对象
        
        # 判断输入是不是列表
        if isinstance(target_arms_pose,arm.Arm_pose):
            self.target_arms_pose    = [target_arms_pose]
        else: # 是
            self.target_arms_pose    = target_arms_pose
        
        # 判断输入是不是列表
        if isinstance(target_clamps_status,arm.GripMethod):
            self.target_clamps_status = [target_clamps_status]
        else:
            self.target_clamps_status = target_clamps_status
            
        self.clamp_speed         = clamp_speed          # 夹具速度
        self.arm_move_method     = arm_move_method      # 移动方式
        self.click_length        = click_length         # 点击长度
    
    # 设置目标位置
    def set_target_arm_pose(self,arm_pose:arm.Arm_pose,device_id:utilis.Device_id):
        if len(self.target_arms_pose) == 2:
            if device_id == utilis.Device_id.LEFT:
                self.target_arms_pose[0] = arm_pose
            elif device_id == utilis.Device_id.RIGHT:
                self.target_arms_pose[1] = arm_pose
        else:
            self.target_arms_pose[0] = arm_pose
            
    # 修改目标xyz
    def modify_target_xyz(self,arm_pose:List[float],device_id:utilis.Device_id):
        if len(self.target_arms_pose) == 2:
            if device_id == utilis.Device_id.LEFT:
                self.target_arms_pose[0].arm_pose[0] = arm_pose[0]
                self.target_arms_pose[0].arm_pose[1] = arm_pose[1]
                self.target_arms_pose[0].arm_pose[2] = arm_pose[2]
            elif device_id == utilis.Device_id.RIGHT:
                self.target_arms_pose[1].arm_pose[0] = arm_pose[0]
                self.target_arms_pose[1].arm_pose[1] = arm_pose[1]
                self.target_arms_pose[1].arm_pose[2] = arm_pose[2]
        else:
            self.target_arms_pose[0].arm_pose[0] = arm_pose[0]
            self.target_arms_pose[0].arm_pose[1] = arm_pose[1]
            self.target_arms_pose[0].arm_pose[2] = arm_pose[2]
        
    # 修改目标xy
    def modify_target_xy(self,arm_pose:List[float],device_id:utilis.Device_id):
        if len(self.target_arms_pose) == 2:
            if device_id == utilis.Device_id.LEFT:
                self.target_arms_pose[0].arm_pose[0] = arm_pose[0]
                self.target_arms_pose[0].arm_pose[1] = arm_pose[1]
            elif device_id == utilis.Device_id.RIGHT:
                self.target_arms_pose[1].arm_pose[0] = arm_pose[0]
                self.target_arms_pose[1].arm_pose[1] = arm_pose[1]
        else:
            self.target_arms_pose[0].arm_pose[0] = arm_pose[0]
            self.target_arms_pose[0].arm_pose[1] = arm_pose[1]
            
    # 修改目标yz
    def modify_target_yz(self,arm_pose:List[float],device_id:utilis.Device_id):
        if len(self.target_arms_pose) == 2:
            if device_id == utilis.Device_id.LEFT:
                self.target_arms_pose[0].arm_pose[1] = arm_pose[1]
                self.target_arms_pose[0].arm_pose[2] = arm_pose[2]
            elif device_id == utilis.Device_id.RIGHT:
                self.target_arms_pose[1].arm_pose[1] = arm_pose[1]
                self.target_arms_pose[1].arm_pose[2] = arm_pose[2]
        else:
                self.target_arms_pose[0].arm_pose[1] = arm_pose[1]
                self.target_arms_pose[0].arm_pose[2] = arm_pose[2]
    
    # 修改xyz并选择机械臂
    def modify_xyz_select_arm(self,arm_pose:List[float],device_id:utilis.Device_id):
        self.modify_target_xyz(arm_pose,device_id)
        self.select_arm(device_id)
    
    # 选择机械臂
    def select_arm(self,device_id:utilis.Device_id,arm_move_method = None):
        if device_id == utilis.Device_id.LEFT:
            self.arm_id = utilis.Device_id.LEFT
            self.target_arms_pose.pop(1)
        elif device_id == utilis.Device_id.RIGHT:
            self.arm_id = utilis.Device_id.RIGHT
            self.target_arms_pose.pop(0)
            
        if arm_move_method != None:
            self.arm_move_method = arm_move_method
        
    # 打印字符串
    def __str__(self) -> str:
        for i in range(len(self.target_arms_pose)):
            if i == 0:
                arms_pose_str = f"Arm_pose_{i}: {self.target_arms_pose[i]} "
            else:
                arms_pose_str = arms_pose_str+ "\n" +  23 * " " + f"Arm_pose_{i}: {self.target_arms_pose[i]} "
        
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
            f"arm_move_method     : {self.arm_move_method} \r\n"
            f"click_length        : {self.click_length } \r\n"
        )

# 任务队列
class Task_sequence():
    def __init__(self, name="") -> None:
        self.task_list : list[Task] = []
        self.name    = name
        
    # 打印输出
    def __str__(self) -> str:
        task_str = ""
        for task in self.task_list:
            task_str = task_str + f"\r\n\r\n{task}"
        return task_str
    
    # 添加任务
    def add(self,task,del_last_naviagte_to_init_point=False):
        # 删除最后一个导航到初始点的任务
        if del_last_naviagte_to_init_point == True:
            if len(self.task_list) > 0:
                if self.task_list[-1].task_type == Task_type.Task_navigate.Navigate_to_the_init_point:
                    self.task_list.pop()
        # 单任务
        if isinstance(task,Task):
            self.task_list.append(task)
        # 增加任务列表
        elif isinstance(task,Task_sequence):
            self.task_list.extend(task.task_list)
        
        self._log_info()
    
    # 更新组id和任务index
    def update_group_id(self,group_id):
        for index,task in enumerate(self.task_list):
            task.set_task_group_id_and_index(group_id,index)
    
    # 弹出第一个任务
    # def pop(self):
    #     task = self.task_list.pop(0)
    #     self._log_info()
    #     return task
    
    # 是否完成, 主要用于并行任务分析的前置任务
    def has_been_done(self):
        for task in self.task_list:
            # 如有一个任务未完成,则返回False
            if task.get_status() != Task.Task_status.FINISHED:
                return False
        return True

    # 获取任务数量
    def get_task_count(self):
        return len(self.task_list)

    # 删除任务
    def remove_task(self,task:Task):
        self.task_list.remove(task)
        self._log_info()
        
    # 打印信息
    def _log_info(self):
        if self.name != "":
            log.log_update_tasks_info(self,self.name)
    
    # 在最后一个任务后面添加返回起始点的任务
    # def add_navigate_to_init_point(self):
    #     task = task.Task_navigation(task.Task_type.Task_navigate.Navigate_to_the_init_point,None,system.anchor_point.map_initial_pose)
    #     self.add(task)

# 任务执行器中的任务管理器
class Task_manager_in_running():
    def __init__(self) -> None:
        self.task_dict = {}

    def add_task(self,task:Task):
        self.task_dict[task.task_index] = task
        return task.task_index
    
    def get_task(self,index=0)->Task:
        task = self.task_dict.get(index)
        return task
    
    def del_task(self,index):
        task = self.task_dict.pop(index)
    

