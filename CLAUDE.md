# CLAUDE.md — 咕嘎导航 (guganav) 项目概述

## 项目简介

北京航空航天大学 **Transistor 战队** RoboMaster **27 赛季哨兵机器人**导航系统。  
基于 **ROS 2 Humble**，C++17，Apache 2.0 许可证。  
GitHub: `https://github.com/LemperorD/guganav`

> 命名来源："咕咕嘎嘎🐧也能学会的导航代码仓库"

---

## 开发环境

| 项目 | 值 |
|------|-----|
| ROS2 发行版 | Humble (`/opt/ros/humble`) |
| C++ 标准 | C++17 (`CMAKE_CXX_STANDARD 17`) |
| 构建工具 | colcon (`scripts/colconBuild.sh`) |
| 代码格式化 | clang-format (基于 Google 风格, 列宽 80, 缩进 2) |
| CI | GitHub Actions (`ros:humble-ros-base` 容器) |
| Pre-commit | clang-format + 三个核心包测试 |
| 仿真器 | Gazebo / Ignition (rmoss 框架) |
| 本地依赖 | `/home/ld/3rdparty/` (Pangolin 0.9.4, OpenCV, Ceres, g2o, Sophus, Livox-SDK) |

---

## 项目结构

```
guganav/
├── CLAUDE.md                  # 本文件
├── README.md                  # 项目介绍（用户维护）
├── .clang-format              # 格式化配置
├── .clang-tidy                # 静态分析配置
├── .pre-commit-config.yaml    # pre-commit 钩子
│
├── docs/
│   └── CODING_STANDARD.md     # 编码规范（模式 A / 模式 B）
│
├── scripts/                   # 工具脚本
│   ├── colconBuild.sh         # 构建
│   ├── simulation.sh          # 仿真启动
│   ├── gitPush.sh             # Git 提交助手
│   ├── nav_decision.sh        # 导航+决策启动
│   ├── map.sh / save_map.sh   # 地图操作
│   ├── pre-commit/            # Pre-commit 测试脚本
│   └── test/                  # 覆盖率测试脚本
│
├── src/
│   ├── guga_bringup/          # [总启动] launch/map/pcd/rviz/behavior_tree/config
│   ├── guga_interfaces/       # [接口] 自定义 ROS2 msg/srv/action
│   ├── guga_description/      # [模型] URDF/Xacro 机器人描述
│   │
│   ├── guga_driver/           # [驱动层]
│   │   ├── serial_driver/     #   串口通信 (BR 协议, 与 MCU 通信)
│   │   ├── livox_ros_driver2/ #   览沃激光雷达
│   │   ├── hik_camera_ros2_driver/ # 海康相机
│   │   ├── hik_driver/        #   海康相机(新版,开发中)
│   │   ├── usbjs_driver/      #   USB 手柄
│   │   └── camera_interface/  #   相机通用接口
│   │
│   ├── guga_perception/       # [感知层]
│   │   ├── terrain_analysis/  #   地形可通行性分析 ★模式A范例
│   │   ├── pointcloud_to_laserscan/ # 点云转激光扫描
│   │   ├── sensor_scan_generation/  # 传感器扫描生成
│   │   ├── loam_interface/    #   LOAM 里程计接口
│   │   └── loam_interface_gravity/  # 带重力对齐的 LOAM 接口
│   │
│   ├── guga_localization/     # [定位层]
│   │   ├── point_lio/         #   Point-LIO 激光惯性里程计
│   │   └── small_gicp_relocalization/ # GICP 重定位
│   │
│   ├── guga_controller/       # [控制层]
│   │   ├── pb_omni_pid_pursuit_controller/ # 全向 PID 追迹控制器 ★模式B范例
│   │   └── fake_vel_transform/  # 速度变换适配
│   │
│   ├── guga_planner/          # [规划层]
│   │   ├── pb_nav2_plugins/   #   Nav2 行为/层插件 (back_up, intensity_voxel)
│   │   └── minco_smoother/    #   MINCO 轨迹平滑器
│   │
│   ├── guga_decision/         # [决策层]
│   │   ├── simple_decision/   #   简单决策系统 (状态机: 默认/攻击/补给)
│   │   └── BehaviorTree.ROS2/ #   行为树框架（备用）
│   │
│   ├── guga_sim/              # [仿真]
│   │   ├── rmoss_core/        #   RMOSS 仿真核心
│   │   ├── rmoss_gazebo/      #   RMOSS Gazebo 插件
│   │   ├── rmoss_gz_resources/#   仿真资源 (模型/世界)
│   │   ├── rmoss_interfaces/  #   仿真接口定义
│   │   ├── rmu_gazebo_simulator/ # RMU 比赛仿真器
│   │   ├── ign_sim_pointcloud_tool/ # Ignition 点云工具
│   │   ├── joint_state_publisher/   # 关节状态发布
│   │   └── sdformat_tools/   #    SDFormat 工具
│   │
│   ├── guga_ui/               # [UI] (开发中, feature_serial 分支)
│   │   ├── guga_ui_common/    #   共享内存通信库 (header-only, 无 ROS 依赖)
│   │   └── guga_ui_pangolin/  #   Pangolin 3D 可视化 (独立进程, 读 shm)
│   │
│   ├── guga_vision/           # [视觉] (预留，当前为空)
│   └── communication_OLD/     # [废弃] 旧版串口通信，将被 serial_driver 替代
│
├── build/                     # colcon 构建输出 (含 COLCON_IGNORE)
├── install/                   # colcon 安装输出
└── log/                       # 运行日志
```

---

## 核心架构

### 数据流（实车模式）

```
传感器 (Livox/相机) → perception (地形/扫描) → planner (Nav2) → controller (PID)
                       ↓                              ↓
                  localization (Point-LIO)         decision (状态机)
                       ↓                              ↓
                  tf (odom→base)                 chassis_mode
                       ↓                              ↓
                  guga_bringup (launch 编排) ← serial_driver (MCU 通信)
                                                       ↓
                                                  下位机 (C板)
```

### 两大编码模式

**模式 A — 函数式数据流** (`terrain_analysis` 范例)  
适用的场景：无状态的纯数据变换管线。  
特征：`Config` + `State` 公开 struct，算法为静态方法，`(const Config&, State&)` 显式传参，2 层抽象。

**模式 B — 对象式封装** (`simple_decision` 范例)  
适用的场景：有跨帧状态的决策系统。  
特征：`Config` 不可变 → `Context` 私有状态 → `Snapshot` 只读快照 → `Decision` 实例方法，3 层抽象。

详见 `docs/CODING_STANDARD.md`。

### 串口通信协议 (BR 协议)

- 帧头: `0x42 0x52` ("BR" = Beihang Robotics)
- 格式: `SOF(2B) | CMD(1B) | LEN(1B) | PAYLOAD(nB) | CRC8(1B)`
- 命令码: `0xCD` 运动控制帧 (26B payload), `0xD1` 裁判系统帧 (13B payload)
- CRC8: poly=0x31, init=0xFF, 查表驱动
- 物理层: 115200 8N1, 非阻塞读写, 1kHz 轮询线程, 3s 超时自动重连

### 共享内存 UI 通信 (新增)

UI 进程 (`guga_ui_pangolin`) 不链接 ROS2，通过 POSIX 共享内存读取算法模块直接写入的数据。

```
算法模块 (serial_driver/simple_decision/point_lio)
    │  ShmWriter::write(&data, 64B)
    ▼
┌──────────────────────────┐
│  POSIX shm "guga_ui_shm" │
│  [Header] [Slot×8] [Data]│
│  lock-free, seq 协议      │
└──────────────────────────┘
    │  ShmReader::read(slot_id, &out)
    ▼
Pangolin UI 进程 (60Hz 渲染)
```

数据槽位：`ROBOT_STATUS` / `GAME_STATUS` / `RFID_STATUS` / `DECISION` / `ENEMY` / `ODOM` / `YAW` / `PATH`

---

## 构建系统

### 构建方式

```bash
# 完整构建
colcon build --symlink-install

# 仅构建特定包
colcon build --packages-select serial_driver guga_ui_common guga_ui_pangolin

# 手动脚本
bash scripts/colconBuild.sh
```

### CMake 模式

- **旧包**使用 `ament_cmake_auto` (自动查找依赖): `guga_bringup`, `guga_controller`, `guga_perception/*`, `guga_planner`, `guga_driver/*`
- **新包**使用 `ament_cmake` (手动声明依赖): `serial_driver`, `guga_ui_common`, `guga_ui_pangolin`
- ROS2 组件节点通过 `rclcpp_components_register_node` 注册

### 包依赖关键关系

```
serial_driver
  ├── guga_interfaces  (RobotStatus, GameStatus, RfidStatus)
  ├── rclcpp + rclcpp_components
  └── nlohmann_json

guga_ui_common
  └── (无 ROS 依赖, 仅 POSIX + std::atomic)

guga_ui_pangolin
  ├── guga_ui_common (header-only)
  ├── Pangolin 0.9.4 (自编译, /home/ld/3rdparty/Pangolin/build)
  └── Eigen3 + OpenGL
```

### CI 系统

`.github/workflows/ci.yml`: 在 `ros:humble-ros-base` 容器中运行 clang-format 检查 + terrain_analysis / PID / simple_decision 的单元测试。

---

## 启动方式

### 实车

```bash
ros2 launch guga_nav_bringup reality_launch.py
```

启动：robot_state_publisher → Livox 雷达 → Nav2 导航栈 → 通信节点(可选)

### 仿真

```bash
ros2 launch guga_nav_bringup simulation_launch.py
# 或
bash scripts/simulation.sh
```

启动：Gazebo → 点云转换 → Nav2 → Rviz

### 仅决策+导航

```bash
bash scripts/nav_decision.sh
```

### UI (独立启动)

```bash
# 先确保算法模块在运行 (已经 init ShmWriter)
ros2 run guga_ui_pangolin guga_ui
```

---

## 命名规范速查

| 元素 | 约定 | 示例 |
|------|------|------|
| class/struct | PascalCase | `SerialDriverMain` |
| 函数/方法 | camelCase | `sendDataFrame()` |
| 变量 (局部/参数) | snake_case | `baud_rate_`, `frame_len` |
| 私有成员 | 尾随 `_` | `fd_`, `config_` |
| 编译期常量 | UPPER_CASE | `FRAME_MIN_SIZE`, `CRC8_INIT` |
| enum class 值 | UPPER_CASE | `CHASSIS_FOLLOWED`, `RUNNING` |
| bool | `is_`/`has_`/`should_`/`can_` 前缀 | `is_hp_deduced` |
| 坐标/距离/阈值 | `double` (不用 float) |
| 数组索引 | `size_t` |
| 网格坐标 | `int` |
| 枚举底层类型 | `uint8_t` |

**统一初始化**: 成员变量一律用 `{}`，禁止 `= 0` / `= false`。  
**返回标记**: 查询方法标记 `[[nodiscard]]`。  
**注释**: Doxygen 风格 `/** @brief ... @param ... @return ... */`，关键逻辑段落在 cpp 中写 `//` 注释说明 WHY。

---

## Git 规范

- 分支命名: `feature_*` / `fix_*` / `refactor_*`
- Commit 格式: `<type>(<scope>): <emoji> <subject>` (遵循 [gitmoji](https://gitmoji.dev/))
- 主分支: `main`
- PR 合并: Merge commit (在 GitHub 上完成)
- Pre-commit: 提交前自动运行 clang-format + 核心包测试

---

## 当前开发状态

### 活跃分支

- `feature_serial` — 串口通信重构 (`communication_OLD` → `serial_driver`)

### 正在进行的开发

| 模块 | 状态 | 说明 |
|------|------|------|
| `serial_driver` | 代码完成，待实车测试 | 已拆分为 SerialDriverMain + RosMcuBridge + SerialDriverNode |
| `guga_ui_common` | 代码完成 | 共享内存通信库，待模块集成 |
| `guga_ui_pangolin` | 代码完成 | Pangolin UI 进程，待编译验证 |
| `hik_driver` | 早期开发 | 海康相机新版驱动 |

### 待模块接入 ShmWriter

以下模块需要在现有 publish 调用旁添加 `shm_writer_.write()` 调用（不影响现有逻辑）：

1. `serial_driver_node.cpp` — `publishRefereeData()` + `decodeYaw()` 处
2. `simple_decision.cpp` — `executeAction()` 处
3. `point_lio` / `laserMapping.cpp` — `publish_odometry()` 处

---

## 关键文件索引

| 文件 | 说明 |
|------|------|
| `docs/CODING_STANDARD.md` | 完整编码规范 |
| `src/guga_perception/terrain_analysis/` | 模式 A 范例代码 |
| `src/guga_decision/simple_decision/` | 模式 B 范例代码 |
| `src/guga_bringup/launch/reality_launch.py` | 实车启动入口 |
| `src/guga_bringup/launch/simulation_launch.py` | 仿真启动入口 |
| `src/guga_bringup/config/` | Nav2 参数 (reality/simulation) |
| `src/guga_driver/serial_driver/include/serial_driver/serial_driver_main.hpp` | 串口协议底层 API |
| `src/guga_ui/guga_ui_common/include/guga_ui_common/ui_types.hpp` | UI 共享内存数据结构 |
| `src/guga_ui/guga_ui_common/include/guga_ui_common/shm_writer.hpp` | 共享内存写入端 |
| `src/guga_ui/guga_ui_common/include/guga_ui_common/shm_reader.hpp` | 共享内存读取端 |
