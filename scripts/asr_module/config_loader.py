import yaml

def load_config(config_file='config.yaml'):
    with open(config_file, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)
    return config

# 加载配置
config = load_config()

# 配置中的各个参数
APPID = config['APPID']
APIKey = config['APIKey']
APISecret = config['APISecret']
silence_threshold = config['silence_threshold']
silence_duration = config['silence_duration']
max_initial_wait = config['max_initial_wait']
app_id = config['app_id']
api_key = config['api_key']
