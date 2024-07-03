# ai_interaction_module.py

import requests
import json
import os
import time

def get_access_token():
    """
    使用 API Key，Secret Key 获取access_token
    """
    client_id = "jL0h3ooGPLw73P9EKorcnCkD"
    client_secret = "RyIpyqI4Rq7u0NZXGI9T3y1Qfp9qhpdL"
    url = f"https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id={client_id}&client_secret={client_secret}"

    payload = json.dumps("")
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload)
    return response.json().get("access_token")


def chat_with_ai(messages):
    access_token = get_access_token()
    url = f"https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/completions_adv_pro?access_token={access_token}"

    payload = json.dumps({
        "messages": messages
    })

    headers = {
        'Content-Type': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload)

    if response.status_code == 200:
        result = response.json().get("result")
        return result
    else:
        print("API调用失败:", response.text)
        return None


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
                    json_objects.append(text[start:end + 1])
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


def save_json_to_file(json_content, file_path_base, count):
    """
    将JSON内容保存到文件中
    :param json_content: JSON字符串
    :param file_path_base: 文件路径基础名
    :param count: 第几次对话
    :return: 文件路径列表和JSON对象列表
    """
    json_objects = extract_json_objects(json_content)
    all_files = []
    timestamp = int(time.time())  # 获取当前时间戳
    for i, json_obj in enumerate(json_objects):
        orders = split_orders(json_obj)
        for order in orders:
            file_path = f"{file_path_base}_{count}_{timestamp}_{i+1}.json"
            with open(file_path, 'w', encoding='utf-8') as json_file:
                json.dump(order, json_file, ensure_ascii=False, indent=4)
            print(f"JSON内容已保存到文件 {file_path}")
            all_files.append(file_path)
    return all_files, json_objects


def get_ai_response_as_dict(messages, count):
    """
    获取AI的回答并转换为字典
    :param messages: 聊天记录
    :param count: 第几次对话
    :return: AI的回答（字典形式）
    """
    result = chat_with_ai(messages)

    if result is None:
        return None, None, messages

    # 生成文件路径基础名
    json_dir = "ai_responses"
    if not os.path.exists(json_dir):
        os.makedirs(json_dir)
    file_path_base = os.path.join(json_dir, f"order")

    # 保存JSON并返回文件路径列表和JSON对象
    files, json_objects = save_json_to_file(result, file_path_base, count)
    if json_objects:
        return files, json.loads(json_objects[0]), result  # 返回第一个JSON对象的字典形式
    return files, None, messages
