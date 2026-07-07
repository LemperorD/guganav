import os

# 使用该命令获取环境变量的值
# 如果环境变量不存在，返回默认值'Default Value'
value = os.environ.get('ACADOS_SOURCE_DIR', 'Default Value')

print(f"ACADOS_SOURCE_DIR: {value}")

# 读取当前文件的绝对路径
print(f"Current file path: {os.path.abspath(__file__)}")
