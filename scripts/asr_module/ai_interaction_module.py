import json
import os
import re
import time
from http import HTTPStatus
from dashscope import Application
from config_loader import app_id, api_key

LOGDIR = "/home/elephant/xzc_code/ros_ws/src/run_task/log/"

def chat_with_ai(messages):
    response = Application.call(app_id=app_id,
                                prompt=messages,
                                api_key=api_key,
                                )

    if response.status_code != HTTPStatus.OK:
        print('request_id=%s, code=%s, message=%s\n' % (response.request_id, response.status_code, response.message))
    else:
        # 直接从 response.output 中提取 text 字段
        result = response.output.get("text", "No text found")
        return result

def remove_comments(json_string):
    pattern = re.compile(r'//.*')
    return re.sub(pattern, '', json_string)

def extract_json_objects(text):
    """
    从文本中提取所有JSON对象
    :param text: 包含JSON对象的文本
    :return: JSON对象列表
    """
    json_objects = []
    start = 0
    while True:
        start = text.find('{', start)
        if start == -1:
            break
        end = start
        brace_count = 0
        while end < len(text):
            if text[end] == '{':
                brace_count += 1
            elif text[end] == '}':
                brace_count -= 1
                if brace_count == 0:
                    json_obj_str = text[start:end + 1]
                    json_obj_str = remove_comments(json_obj_str)

                    # 去除前后的双引号
                    if json_obj_str.startswith('"') and json_obj_str.endswith('"'):
                        json_obj_str = json_obj_str[1:-1]

                    json_objects.append(json_obj_str)
                    break
            end += 1
        start = end + 1
    return json_objects

def replace_empty_json(json_object):
    """
    如果JSON对象是空的，则替换为指定的结构
    :param json_object: JSON对象（字典形式）
    :return: 替换后的JSON对象
    """
    # if not json_object:  # 检查JSON对象是否为空
    if json_object == {}:  # 检查JSON对象是否为空
        return {
            "order_operation": 0,
            "table_id": 0,
            "snacks": {
                "1": 0,
                "2": 0,
                "3": 0,
                "4": 0
            },
            "drinks": {
                "1": 0
            }
        }
    return json_object

def split_orders(json_content):
    """
    拆分orders数组为多个独立的JSON对象
    :param json_content: 复杂的JSON字符串
    :return: 独立的JSON对象列表
    """
    data = json.loads(json_content)
    if "orders" in data:
        return data["orders"]
    else:
        return [data]

def save_json_to_file(json_content, base_dir, count):
    """
    将JSON内容保存到文件中
    :param json_content: JSON字符串
    :param base_dir: 基础目录路径
    :param count: 第几次对话
    :return: 文件路径列表和JSON对象列表
    """
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    json_objects = extract_json_objects(json_content)
    all_files = []
    for i, json_obj in enumerate(json_objects):
        try:
            orders = split_orders(json_obj)
        except json.JSONDecodeError as e:
            print(f"JSONDecodeError: {e}")
            print(f"Invalid JSON: {json_obj}")
            continue
        for order in orders:
            order = replace_empty_json(order)
            file_path = os.path.join(base_dir, f"order_{count}_{i + 1}.json")
            with open(file_path, 'w', encoding='utf-8') as json_file:
                json.dump(order, json_file, ensure_ascii=False, indent=4)
            print(f"JSON内容已保存到文件 {file_path}")
            all_files.append(file_path)
    return all_files, json_objects

def save_result_to_file(result_content, base_dir, count):
    """
    将结果内容保存到文件中
    :param result_content: 结果内容字符串
    :param base_dir: 基础目录路径
    :param count: 第几次对话
    :return: 文件路径
    """
    # 确保基础目录存在
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)

    # 生成文件名并保存内容
    file_path = os.path.join(base_dir, f"order_{count}.txt")
    with open(file_path, 'w', encoding='utf-8') as txt_file:
        txt_file.write(result_content)
    print(f"ai回答内容已保存到文件 {file_path}")

    return file_path

def get_ai_response_as_dict(messages, count, start_timestamp):
    """
    获取AI的回答并转换为字典
    :param messages: 聊天记录
    :param count: 第几次对话
    :param start_timestamp: 程序运行开始时间
    :return: AI的回答（字典形式）
    """
    start_time = time.time()  # 记录开始时间
    result = chat_with_ai(messages)
    if result is None:
        return None, None, result

    end_time = time.time()  # 记录结束时间
    processing_time = end_time - start_time
    print(f"AI处理时间: {processing_time:.2f} 秒")

    # 保存结果到TXT文件
    # save_result_to_file(result, os.path.join("ai_responses", start_timestamp), count)
    save_result_to_file(result, os.path.join(LOGDIR + "/ai_responses", start_timestamp), count)
    
    # 保存JSON并返回文件路径列表和JSON对象
    # files, json_objects = save_json_to_file(result, os.path.join("json_files", start_timestamp), count)
    files, json_objects = save_json_to_file(result, os.path.join(LOGDIR + "/json_files", start_timestamp), count)
    if json_objects:
        try:
            first_json_object = json.loads(json_objects[0])
        except json.JSONDecodeError as e:
            print(f"JSONDecodeError: {e}")
            print(f"Invalid JSON: {json_objects[0]}")
            first_json_object = None
        return files, first_json_object, result  # 返回第一个JSON对象的字典形式
    return files, None, result
