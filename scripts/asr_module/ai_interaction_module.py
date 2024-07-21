import json
import os
import re
import time
from http import HTTPStatus
import dashscope
from dashscope import Generation

# 设置API密钥
dashscope.api_key = "sk-7a78d83cef354876abead0b8d1e7d18b"

def chat_with_ai(messages):
    response = Generation.call(model="qwen-turbo",
                               messages=messages,
                               result_format='message')
    if response.status_code == HTTPStatus.OK:
        result = response.output.choices[0]['message']['content']
        return result
    else:
        print('Request id: %s, Status code: %s, error code: %s, error message: %s' % (
            response.request_id, response.status_code,
            response.code, response.message
        ))
        return None

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
                    json_objects.append(json_obj_str)
                    break
            end += 1
        start = end + 1
    return json_objects

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
    save_result_to_file(result, os.path.join("ai_responses", start_timestamp), count)

    # 保存JSON并返回文件路径列表和JSON对象
    files, json_objects = save_json_to_file(result, os.path.join("json_files", start_timestamp), count)
    if json_objects:
        try:
            first_json_object = json.loads(json_objects[0])
        except json.JSONDecodeError as e:
            print(f"JSONDecodeError: {e}")
            print(f"Invalid JSON: {json_objects[0]}")
            first_json_object = None
        return files, first_json_object, result  # 返回第一个JSON对象的字典形式
    return files, None, result
