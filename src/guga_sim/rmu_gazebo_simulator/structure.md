这是一个 ROS 2 机器人仿真项目，基于 Gazebo 仿真器。以下是各部分的详细功能解释：

## 🏗️ **项目结构总览**

### **根目录文件**
- `dependencies.repos` - 定义项目依赖的 ROS 2 包
- `Dockerfile` - 容器化部署配置
- `README.md` - 项目说明文档

## 📁 **rmu_gazebo_simulator 主包**

### **1. config/ - 配置文件目录**
- `base_params.yaml` - 基础参数配置（机器人控制、导航参数）
- `gz_world.yaml` - Gazebo 世界配置
- `referee_system_1v1.yaml` - 1v1 裁判系统配置
- `ros_gz_bridge.yaml` - ROS 与 Gazebo 桥接配置

### **2. launch/ - 启动文件目录**
- `gazebo.launch.py` - 启动 Gazebo 仿真环境
- `spawn_robots.launch.py` - 在仿真中生成机器人
- `bringup_sim.launch.py` - 完整仿真系统启动（包含所有组件）
- `referee_system.launch.py` - 启动裁判系统
- `rviz.launch.py` - 启动 RViz 可视化工具

### **3. resource/ - 资源文件**
#### **models/ - 地图模型**
- `rmuc_2024/2025` - RMUC（RoboMaster 大学挑战赛）机器人模型
- `rmul_2024/2025` - RMUL（RoboMaster 大学联赛）机器人模型
- 每个模型包含：网格文件(.stl/.dae)、纹理、SDF 模型描述

#### **worlds/ - 仿真世界场景**
- 各种比赛场地：`rmuc_2024_world.sdf`、`rmul_2025_world.sdf`等
- `empty_world.sdf` - 空世界用于测试

### **4. scripts/ - 核心功能脚本**

#### **🔧 player_web/ - 选手控制网页系统**
- Web 界面控制机器人
- `main_vision.py` - 带视觉功能的控制程序
- `main_no_vision.py` - 无视觉功能的控制程序
- `ros_handler.py` - ROS 消息处理
- `static/` - 网页前端资源（CSS、JS、图片）

#### **⚖️ referee_system/ - 裁判系统**
- `referee_system_client.py` - 裁判系统客户端
- `simple_competition_1v1.py` - 1v1 比赛逻辑

#### **📊 referee_web/ - 裁判控制面板**
- Web 界面的裁判系统
- 用于监控比赛状态和计分

#### **🛠️ 工具脚本**
- `killgazebo.sh` - 强制关闭 Gazebo 进程
- `update_model.sh` - 更新机器人模型

### **5. 其他重要文件**
- `rviz/visualize.rviz` - RViz 可视化配置
- `env-hooks/` - 环境钩子脚本

## 📁 **scripts/ - 外部脚本**
- `follow.sh` - 跟随脚本（可能是机器人跟随行为）
- `sim.sh` - 快速启动仿真的便捷脚本

## 🎯 **系统工作流程**

1. **启动仿真** → `gazebo.launch.py` + `spawn_robots.launch.py`
2. **启动控制系统** → `player_web/` 提供 Web 控制界面
3. **启动裁判系统** → 监控比赛状态和计分
4. **可视化** → RViz 实时显示机器人状态和传感器数据

这是一个完整的 **RoboMaster 机器人比赛仿真系统**，支持多机器人对抗、Web 远程控制和自动裁判系统。