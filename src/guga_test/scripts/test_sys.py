import os

# 如果环境变量不存在，返回默认值'Default Value'
value = os.environ.get('ACADOS_SOURCE_DIR', 'Default Value')

print(f"ACADOS_SOURCE_DIR: {value}")