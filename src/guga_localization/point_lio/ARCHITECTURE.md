# Point-LIO 源码架构文档

> **目标读者**: 需要对该代码库进行整理和重构的开发者  
> **生成日期**: 2026-06-24  
> **基础版本**: HKU-MARS Point-LIO + RM25_SMBU_auto_sentry 分支 (prior PCD map)  
> **ROS2 发行版**: Humble

---

## 目录

1. [概述](#1-概述)
2. [系统架构](#2-系统架构)
3. [数据流](#3-数据流)
4. [文件清单与职责](#4-文件清单与职责)
5. [核心算法详解](#5-核心算法详解)
6. [模块依赖关系](#6-模块依赖关系)
7. [两种 EKF 模式对比](#7-两种-ekf-模式对比)
8. [全局状态与数据共享](#8-全局状态与数据共享)
9. [配置参数全景](#9-配置参数全景)
10. [已知问题与重构建议方向](#10-已知问题与重构建议方向)
11. [第三方依赖](#11-第三方依赖)

---

## 1. 概述

Point-LIO 是一个 **高带宽激光惯性里程计 (LiDAR-Inertial Odometry)**，核心特点：

- **逐点处理**: 在每个激光点到达时立即进行 EKF 更新，而非按帧批量处理
- **流形上的迭代卡尔曼滤波**: 基于 IKFoM (Invariant Kalman Filter on Manifolds) 模板库
- **IMU 饱和鲁棒**: 自动检测并舍弃饱和的 IMU 轴数据
- **无运动畸变**: 逐点处理天然消除了帧内运动畸变
- **两种 IMU 模式**: IMU 作为系统输入 (24维) 或作为量测输出 (30维)
- **多雷达支持**: Livox Avia/Horizon/Mid-360, Velodyne VLP-16, Ouster OS1-64, 禾赛 XT32
- **先验地图模式**: 支持加载预构建 PCD 地图进行重定位 (RM25_SMBU_auto_sentry 分支新增)

| 指标 | 值 |
|------|-----|
| 状态维度 | 24 (input) / 30 (output) |
| 最高里程计频率 | 4-8 kHz (受限于点频) |
| 最大角速度 | ~75 rad/s (测试值) |
| 地图数据结构 | iVox 增量体素 |
| 量测模型 | 点到 IMLS 平面距离 |
| C++ 标准 | C++17 |

---

## 2. 系统架构

```
┌──────────────────────────────────────────────────────┐
│                    ROS2 Node                         │
│              pointlio_mapping (500Hz)                 │
├──────────────────────────────────────────────────────┤
│                                                      │
│  ┌──────────┐   ┌──────────┐   ┌───────────────┐   │
│  │ LiDAR    │   │   IMU    │   │  parameters   │   │
│  │ callback │   │ callback │   │  (YAML→全局)   │   │
│  └────┬─────┘   └────┬─────┘   └───────────────┘   │
│       │               │                              │
│       ▼               ▼                              │
│  ┌─────────────────────────────┐                    │
│  │   li_initialization         │                    │
│  │   (sync_packages)           │                    │
│  │   传感器同步 + 数据打包      │                    │
│  └──────────┬──────────────────┘                    │
│             │ MeasureGroup                           │
│             ▼                                        │
│  ┌─────────────────────────────┐                    │
│  │   ImuProcess                │                    │
│  │   IMU 初始化 + 去畸变(预留)  │                    │
│  └──────────┬──────────────────┘                    │
│             │                                        │
│             ▼                                        │
│  ┌─────────────────────────────┐                    │
│  │   VoxelGrid 降采样           │                    │
│  │   + time_compressing 分组    │                    │
│  └──────────┬──────────────────┘                    │
│             │                                        │
│             ▼                                        │
│  ┌─────────────────────────────────┐                │
│  │   Estimator (ESKF 核心)          │                │
│  │   ┌─────────────────────────┐   │                │
│  │   │ get_f / df_dx (预测)     │   │                │
│  │   │ h_model (量测)            │   │                │
│  │   │ process_noise_cov (Q/P)  │   │                │
│  │   └─────────────────────────┘   │                │
│  └──────────┬──────────────────────┘                │
│             │                                        │
│             ▼                                        │
│  ┌─────────────────────────────┐                    │
│  │   MapIncremental             │                    │
│  │   iVox 增量地图更新           │                    │
│  └──────────┬──────────────────┘                    │
│             │                                        │
│             ▼                                        │
│  ┌─────────────────────────────┐                    │
│  │   发布                       │                    │
│  │   odometry / path / TF /    │                    │
│  │   pointcloud / PCD save     │                    │
│  └─────────────────────────────┘                    │
│                                                      │
│  ┌─────────────────────────────┐                    │
│  │   Preprocess (独立模块)       │                    │
│  │   在 LiDAR 回调中同步调用     │                    │
│  │   avia/velodyne/ouster/hesai│                    │
│  └─────────────────────────────┘                    │
└──────────────────────────────────────────────────────┘
```

### 架构分层

| 层 | 模块 | 职责 |
|----|------|------|
| **数据接入** | `li_initialization` | 传感器回调、同步、缓冲区管理 |
| **预处理** | `preprocess` / `IMU_Processing` | 点云特征提取、时间戳计算、IMU初始化 |
| **状态估计** | `Estimator` + IKFoM | ESKF 预测/更新、量测模型、雅可比 |
| **地图管理** | `ivox` | 增量体素地图、近邻搜索 |
| **输出** | `laserMapping` (main) | 发布里程计/路径/点云/TF、日志 |
| **配置** | `parameters` | YAML参数解析、全局变量 |

---

## 3. 数据流

### 3.1 传感器数据流入

```
LiDAR 驱动 ──→ livox_pcl_cbk / standard_pcl_cbk ──→ Preprocess ──→ lidar_buffer
IMU 驱动  ──→ imu_cbk (时间校准)                   ──→ imu_deque
```

**时间戳约定**: 点云的 `curvature` 域复用存储时间偏移 (单位: ms)。帧起始时间存于 `time_buffer`，点的绝对时间 = 帧起始时间 + curvature/1000。

### 3.2 主循环处理 (500Hz)

```
executor.spin_some()    ← 非阻塞回调处理
        │
        ▼
sync_packages()         ← 等待一组 LiDAR+IMU 数据就绪
        │
        ▼ (首帧初始化: 重力设置, IMU 时间对齐)
        │
p_imu->Process()        ← IMU 初始化或点云去畸变
        │
        ▼
VoxelGrid 降采样        ← 体素滤波器 (filter_size_surf_min)
        │
        ▼
time_compressing()      ← 按时间戳分组点云
        │
        ▼
[地图初始化]             ← 累积 init_map_size 点
        │                   (或加载 prior PCD)
        ▼
EKF 逐点迭代:           ← 对每组点:
  predict(dt, Q)        ←   IMU 状态传播
  update(h_x, z)        ←   点面残差更新
  update_IMU()          ←   IMU伪量测更新 (output模式)
        │
        ▼
pointBodyToWorld()      ← 更新后状态变换所有点到世界系
        │
        ▼
MapIncremental()        ← 新点加入 iVox 地图
        │
        ▼
publish_odometry()      ← 发布里程计
publish_path()          ← 发布路径
publish_frame_world()   ← 发布配准点云
publish_frame_body()    ← 发布 IMU 系点云
```

### 3.3 逐点处理的时间线

一帧 LiDAR 点云 (例如 10Hz, 数千点) 被 `time_compressing()` 按时间戳分组。对每组点：

1. 根据 IMU 数据向前预测状态到当前组的时间
2. 将该组点变换到世界坐标系
3. 在 iVox 地图中搜索最近邻，拟合平面，计算点面残差
4. 迭代更新状态
5. 重复处理下一组

这实现了**逐点/逐组 EKF 更新**，消除了运动畸变并实现了高带宽输出。

---

## 4. 文件清单与职责

### 4.1 核心源码 (`src/`)

| 文件 | 行数 | 角色 | 关键内容 |
|------|------|------|---------|
| `laserMapping.cpp` | 1196 | **主入口** | `main()`, 发布函数, 地图初始化/增量, 日志 |
| `parameters.cpp` | 413 | 参数配置 | `readParameters()`, 全局变量定义, 协方差重置 |
| `parameters.h` | 210 | 参数声明 | 所有全局 `extern` 变量 (120+), 函数声明 |
| `Estimator.cpp` | 614 | ESKF 核心 | 过程噪声 Q, f(x,u), df/dx, h_model, pointBodyToWorld |
| `Estimator.h` | 232 | ESKF 声明 | 量测变量, 10个函数声明 |
| `preprocess.cpp` | 1058 | 点云预处理 | 4种 handler, give_feature, plane_judge, edge_jump_judge |
| `preprocess.h` | 319 | 预处理声明 | Preprocess类, 枚举, orgtype结构, 雷达原生点结构 |
| `li_initialization.cpp` | 378 | 传感器同步 | 3种回调, sync_packages, 合帧/切帧 |
| `li_initialization.h` | 148 | 同步声明 | 缓冲区, 互斥变量, 调试数组 |
| `IMU_Processing.cpp` | 190 | IMU 处理 | IMU_init (滑动平均), Set_init (重力对齐), Process |
| `IMU_Processing.h` | 126 | IMU 声明 | ImuProcess类 |

### 4.2 公共头文件 (`include/`)

| 文件 | 行数 | 角色 |
|------|------|------|
| `common_lib.h` | 345 | 公共类型: 流形定义、常量、宏、MeasureGroup、平面估计工具函数 |
| `so3_math.h` | 219 | SO(3) 数学: Exp, Log, 反对称矩阵, 欧拉角, 右雅可比逆 |

### 4.3 第三方头文件 (`include/`)

| 目录/文件 | 行数 | 来源 | 说明 |
|-----------|------|------|------|
| `IKFoM/` | ~3000 | HKU-MARS | 流形上的不变卡尔曼滤波模板库 (header-only) |
| `ivox/` | ~1500 | Faster-LIO | 增量体素地图 (Hilbert曲线, iVox3D) |
| `matplotlibcpp.h` | 2659 | lava/matplotlib-cpp | C++ matplotlib 绑定 (调试绘图用, 实际未使用) |

### 4.4 配置文件

| 文件 | 说明 |
|------|------|
| `config/mid360.yaml` | Mid-360 配置 (默认) |
| `config/avia.yaml` | Avia 配置 |
| `config/horizon.yaml` | Horizon 配置 |
| `config/velody16.yaml` | Velodyne VLP-16 配置 |
| `config/ouster64.yaml` | Ouster OS1-64 配置 |
| `launch/point_lio.launch.py` | ROS2 launch 文件 |
| `rviz_cfg/loam_livox.rviz` | RViz2 可视化配置 |

---

## 5. 核心算法详解

### 5.1 状态估计 (ESKF on Manifold)

**状态空间** (以 input 模式 24 维为例):

| 索引 | 变量 | 含义 | 维度 |
|------|------|------|------|
| 0:3 | pos | 位置 (世界系) | 3 |
| 3:6 | rot | 姿态 SO(3) | 3 |
| 6:9 | offset_R_L_I | LiDAR→IMU 外参旋转 | 3 |
| 9:12 | offset_T_L_I | LiDAR→IMU 外参平移 | 3 |
| 12:15 | vel | 速度 (世界系) | 3 |
| 15:18 | bg | 陀螺仪零偏 | 3 |
| 18:21 | ba | 加速度计零偏 | 3 |
| 21:24 | gravity | 重力向量 (世界系) | 3 |

**动力学模型** (input 模式):

```
d(pos)/dt  = vel
d(rot)/dt  = rot * [ω_meas - bg]×
d(vel)/dt  = rot * (a_meas - ba) + gravity
d(bg)/dt   = 0
d(ba)/dt   = 0
d(gravity)/dt = 0
d(外参)/dt = 0
```

### 5.2 量测模型 (点面距离)

对每个特征点，在 iVox 地图中搜索 5 个最近邻，拟合局部平面 n·p + d = 0：

```
残差:  z = n·p_world + d
雅可比: H = [n^T | A^T | B^T | C^T]

其中:
  C = R^T · n           (世界系法向量 → IMU 系)
  A = [p_imu]× · C      (对姿态误差的偏导)
  B = [p_body]× · R_LI^T · C  (对外参旋转的偏导)
```

**有效点筛选**: Mahalanobis 距离检验: `|p_body|² > match_s · pd2²` (match_s 默认为 81)。

### 5.3 IMU 伪量测 (output 模式)

在 output 模式下，角速度和加速度是状态变量。IMU 数据作为量测：

```
z_IMU[0:3] = ω_meas - (omg + bg)      (角速度残差)
z_IMU[3:6] = a_meas * G/acc_norm - (acc + ba)  (加速度残差)
```

**饱和处理**: 当某轴测量值 ≥ 99%·satu 时，残差置零并标记忽略。

### 5.4 IMU 初始化

累积 `MAX_INI_COUNT` (100) 帧的 IMU 数据，用滑动平均估计：
- **重力方向**: `mean_acc / |mean_acc|` (静态时加速度 ≈ 重力)
- **陀螺仪零偏**: `mean_gyr`

然后通过 `Set_init()` 计算初始姿态使估计重力与先验重力对齐。

### 5.5 特征提取 (机械雷达)

`give_feature()` 对每条扫描线的连续点进行几何分析：

- **平面判定 (plane_judge)**: 首末点连线方向的垂直偏离度 + 距离方差比值
- **跳变边缘 (edge_jump_judge)**: 前后点到激光的距离突变 + 角度连续性判定
- **最终分类**: `Real_Plane`, `Poss_Plane`, `Edge_Jump`, `Edge_Plane`, `Wire`

注意: Livox 非重复扫描雷达不使用此特征提取，所有有效点都作为曲面点输出。

### 5.6 iVox 增量地图

- 基于 Hilbert 曲线的空间索引实现 O(1) 近邻搜索
- 地图降采样分辨率: `filter_size_map_min`
- 近邻搜索类型: CENTER (0) / NEARBY6 (18) / NEARBY18 (默认) / NEARBY26

### 5.7 地图增量更新 (MapIncremental)

对每个世界系曲面点，检查其所在体素网格是否已有地图点：
- 如果已有 (且距离体素中心 < 0.5 * 分辨率): 跳过 (避免冗余)
- 否则: 加入 `points_to_add` 批量添加到 iVox

---

## 6. 模块依赖关系

```
                      ┌─────────────┐
                      │ common_lib.h │ (流形类型, 常量, 工具函数)
                      └──────┬──────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
      ┌──────────┐   ┌────────────┐   ┌──────────┐
      │so3_math.h│   │parameters.h│   │IKFoM/    │
      │          │   │            │   │esekfom   │
      └──────────┘   └─────┬──────┘   └──────────┘
                           │
         ┌─────────────────┼─────────────────┐
         ▼                 ▼                  ▼
  ┌─────────────┐  ┌─────────────┐   ┌──────────────┐
  │preprocess.h │  │IMU_Process.h│   │ Estimator.h  │
  │  (独立)      │  │             │   │              │
  └─────────────┘  └──────┬──────┘   └──────┬───────┘
                          │                  │
                          ▼                  ▼
                   ┌──────────────────────────┐
                   │   li_initialization.h     │
                   │   (引用 Estimator + IMU)  │
                   └────────────┬─────────────┘
                                │
                                ▼
                        ┌───────────────┐
                        │laserMapping.cpp│ (顶层: 引用所有上述模块)
                        └───────────────┘
```

**依赖关系特点**:

- `preprocess` 是独立模块，不依赖其他源码
- `IMU_Processing` 依赖 `common_lib.h`
- `Estimator` 依赖 `common_lib.h` + `parameters.h`
- `li_initialization` 依赖上述所有 + `Estimator`
- `laserMapping.cpp` 是唯一的顶层入口，直接包含 `li_initialization.h`

---

## 7. 两种 EKF 模式对比

| 维度 | input 模式 | output 模式 |
|------|-----------|------------|
| 参数名 | `use_imu_as_input: True` | `use_imu_as_input: False` (默认) |
| 状态维度 | 24 | 30 |
| 过程噪声维度 | 12 (ng, na, nbg, nba) | 15 (+ vel噪声) |
| 量测模型数 | 1 (激光点面) | 2 (激光点面 + IMU伪量测) |
| 角速度/加速度 | 来自 IMU 测量 (去零偏) | 来自状态估计 |
| 状态导数 | d(vel)/dt = R·(a_meas-ba) + g | d(vel)/dt = R·acc + g |
| 适用场景 | IMU 质量好、同步精确 | IMU 可能饱和、需要估计角速度/加速度 |
| 推荐场景 | 常规使用 | 高动态/IMU 易饱和的场景 |

### 7.1 两种模式的状态转移矩阵差异

**input 模式**:
- `F(12:15, 18:21) = -R` (加速度零偏对速度的影响，符号: 负)
- `F(3:6, 15:18) = -I` (陀螺零偏对姿态的影响)

**output 模式**:
- `F(12:15, 18:21) = +R` (直接使用状态 acc，符号: 正)
- `F(3:6, 15:18) = +I` (角速度是状态)

---

## 8. 全局状态与数据共享

### 8.1 全局变量分布

项目使用大量全局变量进行模块间数据共享 (无封装):

| 类别 | 数量 | 声明位置 | 定义位置 |
|------|------|---------|---------|
| 配置参数 | ~60 | `parameters.h` | `parameters.cpp` |
| 状态变量 | ~20 | `parameters.h` | `parameters.cpp` |
| 点云缓存 | ~10 | `Estimator.h` | `Estimator.cpp` |
| EKF 实例 | 2 | `common_lib.h` | `Estimator.cpp` |
| 量测数据 | ~15 | `Estimator.h` | `Estimator.cpp` |
| 同步缓冲区 | ~15 | `li_initialization.h` | `li_initialization.cpp` |

### 8.2 关键全局变量

- `kf_input` / `kf_output`: 两个全局 EKF 实例 (全局单例)
- `Measures`: 当前处理的 MeasureGroup (在 sync_packages 中填充)
- `ivox_`: iVox 地图智能指针
- `feats_down_body` / `feats_down_world`: 降采样后的 IMU 系/世界系点云
- `time_seq`: 时间分组序列
- `Nearest_Points`: 每个点的最近邻缓存
- `Lidar_T_wrt_IMU` / `Lidar_R_wrt_IMU`: 固定外参
- `extrinT` / `extrinR`: YAML 中的外参 (向量形式)

---

## 9. 配置参数全景

### 9.1 参数分类

```
ros__parameters:
├── 模式开关
│   ├── use_imu_as_input       # True=input(24维) / False=output(30维)
│   ├── prop_at_freq_of_imu    # 按IMU频率传播
│   ├── check_satu             # IMU饱和检测
│   ├── space_down_sample      # 二次空间降采样
│   ├── imu_en                 # 是否启用IMU
│   └── extrinsic_est_en       # 是否在线估计外参
│
├── common
│   ├── lid_topic / imu_topic  # 话题名
│   ├── con_frame / con_frame_num  # 合帧
│   ├── cut_frame / cut_frame_time_interval  # 切帧
│   └── time_diff_lidar_to_imu # LiDAR→IMU 时差
│
├── prior_pcd                  # 先验地图 (RM25分支新增)
│   ├── enable                 # 启用先验PCD
│   ├── prior_pcd_map_path     # 地图路径
│   └── init_pose              # 初始位姿 [x,y,z]
│
├── preprocess
│   ├── lidar_type             # 1=AVIA 2=VELO16 3=OUST64 4=HESAIxt32
│   ├── scan_line / scan_rate  # 扫描线数/频率
│   ├── timestamp_unit         # 时间戳单位
│   └── blind                  # 盲区距离
│
├── mapping (核心调优参数)
│   ├── 噪声: acc_cov_*, gyr_cov_*, b_*_cov, vel_cov, lidar_meas_cov
│   ├── 饱和: satu_acc, satu_gyro, acc_norm
│   ├── 平面: plane_thr, match_s
│   ├── 滤波: filter_size_surf, filter_size_map, ivox_grid_resolution
│   ├── 外参: extrinsic_T, extrinsic_R
│   ├── 重力: gravity, gravity_init
│   └── 时序: imu_time_inte, lidar_time_inte
│
├── odometry
│   └── publish_odometry_without_downsample
│
├── publish
│   ├── path_en                # 发布路径
│   ├── scan_publish_en        # 发布配准点云
│   ├── scan_bodyframe_pub_en  # 发布IMU系点云
│   └── tf_send_en             # 发布TF变换
│
└── pcd_save
    ├── pcd_save_en            # PCD保存
    └── interval               # 保存间隔 (帧数, -1=结束时保存)
```

---

## 10. 已知问题与重构建议方向

### 10.1 代码质量问题

| 问题 | 位置 | 描述 |
|------|------|------|
| **全局变量泛滥** | 全项目 | 120+ 全局变量，无封装，模块间隐式依赖 |
| **全局 EKF 实例** | `common_lib.h`/`Estimator.cpp` | `kf_input`/`kf_output` 为全局单例，无法多实例 |
| **宏误用风险** | `common_lib.h` | `VEC_FROM_ARRAY` 等宏展开可能产生未定义行为 |
| **硬编码数组大小** | `Estimator.cpp` | `point_selected_surf[100000]` 固定大小 |
| **重复代码** | `Estimator.cpp` | `h_model_input` 和 `h_model_output` 近乎完全相同 |
| **重复代码** | `laserMapping.cpp` | input/output 模式的主循环分支高度重复 |
| **重复代码** | `li_initialization.cpp` | `standard_pcl_cbk` 和 `livox_pcl_cbk` 大部分逻辑相同 |
| **未使用代码** | 多处 | 被注释的代码块、未使用的变量 (`depth_feats_world`, `s_plot2`, `s_plot3`) |
| **头文件污染** | `parameters.h` | 包含 `<Python.h>` 但不使用，引入了大量无关依赖 |
| **命名不一致** | 多处 | `laserMapping` vs `pointlio_mapping`, `lidar` vs `lidar` |
| **魔数** | `preprocess.cpp` | 大量硬编码的经验参数 (如 `group_size=8`, `p2l_ratio=225`) |
| **处处理函数过长** | `laserMapping.cpp` | `main()` 函数 ~400 行，难以理解和测试 |
| **线程安全** | `li_initialization.cpp` | `mtx_buffer` 定义了但从未使用 (已注释) |
| **IMU 去畸变缺失** | `IMU_Processing.cpp` | `Process()` 仅复制原始点云，未实现反向传播去畸变 |
| **`using namespace std`** | 多个头文件 | 头文件中使用会污染所有包含方的命名空间 |

### 10.2 架构改进方向

1. **消除全局变量 — 引入 Context 类**

   跟随项目编码规范中的**模式 B (对象式封装)**:
   - 创建 `LioContext` 或 `LioEngine` 类封装状态、EKF 实例、地图、参数
   - 将全局变量作为成员变量，在构造函数中初始化
   - 支持多实例 (例如同时运行多个 LIO)

2. **拆分 `laserMapping.cpp`**

   `main()` 函数过于庞大 (400+ 行)，建议拆分为:
   - `LioPipeline`: 封装完整流水线 (predict→update→map→publish)
   - `LioPublisher`: 封装所有发布逻辑
   - `LioInitializer`: 封装首帧初始化、地图初始化逻辑

3. **消除 `h_model_input` / `h_model_output` 重复**

   两个函数逻辑完全相同，区别仅在于通过 `pointBodyToWorld` 间接引用不同的全局 EKF。
   引入泛型模板或策略模式合并。

4. **统一 LiDAR 回调**

   `standard_pcl_cbk` 和 `livox_pcl_cbk` 的合帧/切帧/入队逻辑相同。
   可抽取为模板或统一接口。

5. **参数管理改进**

   - 移除 `parameters.h` 中无关的头文件
   - 用嵌套 struct 替代平铺的全局变量
   - 考虑使用 ROS2 参数回调实现运行时调参

6. **IMU 去畸变实现**

   `ImuProcess::Process()` 中预留了去畸变接口但未实现。
   可参考 FAST-LIO2 的反向传播去畸变方法。

7. **单元测试**

   当前无任何单元测试。重构后应为核心模块添加测试:
   - `Estimator`: 状态转移、雅可比正确性 (数值差分验证)
   - `Preprocess`: 时间戳计算、特征提取
   - `ImuProcess`: 初始化、重力对齐

8. **CMake 现代化**

   - `find_package(PythonLibs REQUIRED)` 是 ROS1 遗留，注释说应换 Pybind11
   - 移除未使用的 `rclpy` 依赖
   - 将 IKFoM/ivox 作为 proper 的 ROS2 包依赖而非 vendored 头文件

### 10.3 重构优先级建议

| 优先级 | 方向 | 收益 | 风险 |
|--------|------|------|------|
| 🔴 高 | 消除全局变量, 引入 Context 类 | 可测试性, 可维护性 | 需要仔细处理数据流 |
| 🔴 高 | 拆分 `laserMapping.cpp` | 可读性 | 低 (纯提取) |
| 🔴 高 | 合并重复的量测模型代码 | 减少50%重复 | 需验证等价性 |
| 🟡 中 | 清理未使用代码和头文件 | 编译速度, 清晰度 | 低 |
| 🟡 中 | 参数结构化管理 | 可配置性 | 需更新 YAML schema |
| 🟢 低 | 实现 IMU 去畸变 | 精度提升 | 需验证算法 |
| 🟢 低 | 添加单元测试 | 回归保护 | 需搭建测试框架 |
| 🟢 低 | CMake 清理 | 构建简洁 | 低 |

---

## 11. 第三方依赖

### 11.1 系统依赖 (apt)

- `libeigen3-dev` (Eigen3 线性代数)
- `libpcl-dev` (PCL 点云库)
- `ros-$ROS_DISTRO-pcl-conversions` (PCL↔ROS2 桥接)
- `libgoogle-glog-dev` (Google 日志库)
- `libunwind-dev` (栈回溯)

### 11.2 Vendor 依赖 (源码内嵌)

| 库 | 路径 | 来源 | 许可证 |
|----|------|------|--------|
| **IKFoM** | `include/IKFoM/` | [HKU-MARS/IKFoM](https://github.com/hku-mars/IKFoM) | GPLv3 |
| **iVox** | `include/ivox/` | [gaoxiang12/faster-lio](https://github.com/gaoxiang12/faster-lio) (Faster-LIO) | BSD |
| **matplotlib-cpp** | `include/matplotlibcpp.h` | [lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp) | MIT |

### 11.3 ROS2 包依赖

- `livox_ros_driver2` (Livox 雷达驱动, 提供 `CustomMsg` 消息类型)
- `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`

---

## 附录

### A. 坐标帧约定

| 帧名 | 含义 | 说明 |
|------|------|------|
| `camera_init` | 世界坐标系 (初始帧) | 第一帧 LiDAR 位姿定义的原点 |
| `body` | IMU 本体坐标系 | 里程计输出的子帧 |
| `aft_mapped` | 建图后位姿 | TF 发布的目标帧 |

### B. curvature 域的复用

点云的 `curvature` 字段全程用于存储**时间偏移** (单位: 毫秒)，而非几何曲率:
- **Livox**: `offset_time (ns) / 1e6`
- **Velodyne**: `time * time_unit_scale`
- **Ouster**: `t * time_unit_scale`
- **禾赛**: `(timestamp - time_head) * time_unit_scale`

### C. 关键默认值速查

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `MAX_INI_COUNT` | 100 | IMU 初始化帧数 |
| `NUM_MATCH_POINTS` | 5 | 平面拟合近邻数 |
| `match_s` | 81 | 马氏距离阈值 (≈9²) |
| `plane_thr` | 0.05 | 平面一致性阈值 |
| `filter_size_surf_min` | 0.5m | 点云降采样分辨率 |
| `filter_size_map_min` | 0.5m | 地图降采样分辨率 |
| `ivox_grid_resolution` | 2.0m (Mid360) / 0.2m (默认) | iVox 体素边长 |
| `laser_point_cov` | 0.01 | 量测噪声方差 |
| `imu_time_inte` | 0.005s | IMU 积分步长 |
| `G_m_s2` | 9.81 | 重力加速度 |
