#! /usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# from __future__ import annotations
import rospy
import os
import sys
import rospkg
from typing import List
rospack = rospkg.RosPack()
package_path = rospack.get_path('run_task')
sys.path.insert(0,package_path + "/scripts")

import actionlib
import utilis
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from enum import Enum,auto # 任务字典
import run_task.msg as msg



# 零食类
class Snack:
    snack_type = {
        "益达口香糖": 1,
        "果冻"  : 2,
        "伊利每益添乳酸菌" : 3,
        "金津陈皮丹" :4
    }
    
    class Snack_id(Enum):
        TBD       = 0
        YIDA      = auto() #1
        # GUOSHU    = auto() #2
        # CKU       = auto() #3
        GUODONG   = auto() #2
        RUSUANJUN = auto() #3
        CHENPIDAN = auto() #4
        
        # def __str__(self) -> str:
        #     return self.value
    
    def __init__(self,id:Snack_id,count:int) -> None:
        self.id    = id
        self.count = count
        
    # 打印
    def __str__(self) -> str:
        return f"id:{self.id.value},name:{self.id.name},count:{self.count}"
    
    # 获取零食类型
    @staticmethod
    def get_snack_id(snack_name:str):
        return Snack.snack_type[snack_name]
    
    # 转为msg
    def to_snack_msg(self):
        return msg.SnackIDWithCount(self.id.value,self.count)
    
    # 获取零食名称
    @staticmethod
    def get_snack_name(snack_id:int):
        for name, id in Snack.snack_type.items():
            if id == snack_id:
                return name
    
    # 转为零食名称 
    def to_snack_name(self):
        return self.get_snack_name(self.id)
    
    # 由消息初始化
    @staticmethod
    def instantialize_from_msg(msg:msg.SnackIDWithCount):
        # rospy.loginfo(f"{rospy.get_name()} instantialize_from_msg")
        snack = Snack(Snack.Snack_id(msg.snack_id),msg.count)
        # rospy.loginfo(f"{rospy.get_name()} snack:{snack}")
        return snack


# 零食列表
class Snack_list():
    def __init__(self) -> None:
        self.snack_list:list[Snack] = []
        
    # 打印
    def __str__(self) -> str:
        return str([str(snack) for snack in self.snack_list])
        
    def __len__(self):
        return len(self.snack_list)
    
    # 重写枚举值
    def __iter__(self):
        self.index = 0  # 每次开始迭代时重置索引
        return self
    
    def __next__(self):
        if self.index < len(self.snack_list):
            snack = self.snack_list[self.index]
            result = [snack.id,snack.count]
            self.index += 1
            return result
        else:
            raise StopIteration 
    
    def add_snack(self,snack:Snack):
        self.snack_list.append(snack)
    
    def to_msg(self):
        msg_snack_list = []
        for snack in self.snack_list:
            msg_snack_list.append(snack.to_snack_msg())
        return msg_snack_list
    
    def to_list(self):
        id_list = []
        for snack in self.snack_list:
            id_list.append(snack.id.value)
        return id_list
    
    # 由消息初始化
    @staticmethod
    def instantialize_from_msg(msg:List[msg.SnackIDWithCount]):
        snack_list = Snack_list()
        for snack_msg in msg:
            snack = Snack.instantialize_from_msg(snack_msg)
            snack_list.add_snack(snack)
        return snack_list
    
    def get_all_snack_count(self):
        count = 0
        for snack in self.snack_list:
            count += snack.count
        return count
    

# 饮料类
class Drink():
    drink_type = {
        "咖啡": 1,
        "雪碧": 2
    }
    
    # 饮料ID
    class Drink_id(Enum):
        TBD    = 0
        COFFEE = auto()
        XUEBI  = auto()
        
        # def __str__(self) -> str:
        #     return self.value
    
    def __init__(self,id:Drink_id,count:int) -> None:
        self.id    = id
        self.count = count
    
    # 打印输出
    def __str__(self) -> str:
        return f"id:{self.id.value},name:{self.id.name},count:{self.count}"
    
    # 获取饮料类型
    @staticmethod
    def get_drink_id(drink_name:str):
        return Drink.drink_type[drink_name]
    
    # 获取饮料名称
    @staticmethod
    def get_drink_name(drink_id:int):
        for name, id in Drink.drink_type.items():
            if id == drink_id:
                return name
            
    # 由消息初始化
    @staticmethod
    def instantialize_from_msg(msg:msg.DrinkIDWithCount):
        # rospy.loginfo(f"{rospy.get_name()} instantialize_from_msg")
        drink = Drink(Drink.Drink_id(msg.drink_id),msg.count)
        # rospy.loginfo(f"{rospy.get_name()} drink:{drink}")
        return drink
    
    # 转为msg
    def to_drink_msg(self):
        return msg.DrinkIDWithCount(self.id.value,self.count)
    


# 饮料列表
class Drink_list():
    def __init__(self) -> None:
        self.drink_list:list[Drink] = []
        
    def __len__(self):
        return len(self.drink_list)
    
    # 重写枚举值
    def __iter__(self):
        self.index = 0  # 每次开始迭代时重置索引
        return self
    
    def __next__(self):
        if self.index < len(self.drink_list):
            drink = self.drink_list[self.index]
            result = [drink.id,drink.count]
            self.index += 1
            return result
        else:
            raise StopIteration 
        
    # 打印输出
    def __str__(self) -> str:
        return str([str(drink) for drink in self.drink_list])
    
    
    # 增加饮料
    def add_drink(self,drink:Drink):
        self.drink_list.append(drink)
    
    # 转为msg
    def to_msg(self):
        msg_drink_list:list[msg.DrinkIDWithCount] = []
        for drink in self.drink_list:
            msg_drink_list.append(drink.to_drink_msg())
        return msg_drink_list
    
    # 由消息初始化
    @staticmethod
    def instantialize_from_msg(msg:List[msg.DrinkIDWithCount]):
        drink_list = Drink_list()
        for drink_msg in msg:
            drink = Drink.instantialize_from_msg(drink_msg)
            drink_list.add_drink(drink)
        return drink_list
    

# 桌子ID类
class Table_id(Enum):
    TBD        = 0        # 待定
    LEFT       = auto()   # 左
    RIGHT      = auto()   # 右
    
# 订单类
class Order():
    # 订单状态, 暂未使用
    class Order_status(Enum):
        WAITING    = 0          # 等待
        DELIVERING = auto()     # 送货中
        FINISHED   = auto()     # 完成
        CANCELED   = auto()     # 取消

    # 订单操作, 增删改查
    class Operation(Enum):
        ADD    = 0              # 增加
        DELETE = auto()         # 删除
        MODIFY = auto()         # 修改
        CHECK  = auto()         # 查询
    
    last_order_id = 0           # 最新订单ID
    
    # 订单信息
    def __init__(self,new_order_id=True) -> None:
        if new_order_id:
            self.order_id = self.last_order_id + 1
            self.last_order_id = self.order_id
        self.snack_list = Snack_list()  # 零食列表
        self.drink_list = Drink_list()  # 饮料列表
        self.table_id   = -1            # 桌号
        
    # 打印输出
    def __str__(self):
        return f"order_id:{self.order_id}, table_id:{self.table_id}, snacks:{self.snack_list}, drinks:{self.drink_list}"
    
    # 订单操作 
    def set_order_operation(self,operation:Operation):
        self.operation = operation.value
    
    # 桌号
    def set_table_id(self,table_id:Table_id):
        self.table_id = table_id.value
        
    # 订单号
    def set_order_id(self,order_id:int):
        self.order_id = order_id
        
    # 添加零食
    def add_snack(self,snack:Snack):
        self.snack_list.add_snack(snack)
        
    # 添加饮料
    def add_drink(self,drink:Drink):
        self.drink_list.add_drink(drink)
    
    #  是否有零食请求
    def has_snack_request(self):
        return len(self.snack_list) > 0
    
    #  是否有饮料请求
    def has_drink_request(self):
        return len(self.drink_list) > 0
    
    # 转为msg
    def to_msg(self):
        order_msg = msg.OrderInfo()
        order_msg.order_id = self.order_id
        order_msg.table_id = self.table_id
        order_msg.order_operation = self.operation
        order_msg.snacks = self.snack_list.to_msg()
        order_msg.drinks = self.drink_list.to_msg()
        return order_msg
    
    # 从消息初始化
    @staticmethod
    def instantialize_from_msg(msg:msg.OrderInfo):
        rospy.loginfo(f"{rospy.get_name()} order instantialize_from_msg")
        order = Order(False)
        order.order_id   = msg.order_id
        order.table_id   = msg.table_id
        order.operation  = Order.Operation(msg.order_operation)
        order.snack_list = Snack_list.instantialize_from_msg(msg.snacks)
        order.drink_list = Drink_list.instantialize_from_msg(msg.drinks)
        rospy.loginfo(f"{rospy.get_name()} order:{order}")
        return order
    
    
    def get_snack_list(self):
        return self.snack_list
    
    def get_drink_list(self):
        return self.drink_list
    
    def get_all_snack_count(self):
        count = 0
        for snack in self.snack_list.snack_list:
            count += snack.count
        return count
    
    def get_all_drink_count(self):
        count = 0
        for drink in self.drink_list.drink_list:
            count += drink.count
        return count
    
