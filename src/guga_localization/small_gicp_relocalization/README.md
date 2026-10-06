# small_gicp_relocalization

## 本仓库适配

本包是 `guganav` 中的 ROS 2 Humble 重定位实现，构建目标为
`small_gicp_relocalization`。节点订阅 Point-LIO 输出的 `registered_scan`，与先验点云
地图做 [small_gicp](https://github.com/koide3/small_gicp) GICP 配准，求出
`map -> odom` 的修正量并以 TF 广播。它依赖源码编译的 `small_gicp`（默认安装到
`/usr/local`），由仓库脚本按固定 commit 安装。

从工作空间根目录安装依赖、构建并启动：

```bash
cd ~/guganav
bash scripts/ci/install_deps_humble.sh   # 安装 small_gicp（固定 commit）、acados 等依赖
source /opt/ros/humble/setup.bash
colcon build --packages-select small_gicp_relocalization
source install/setup.bash
```

包内 `launch/small_gicp_relocalization_launch.py` 只用于单包调试：参数直接写在
Node 的参数表里，且 `prior_pcd_file` 默认为空，单独启动会报
`Couldn't read PCD file`。实车与仿真都由 `guga_bringup` 的定位 launch 启动，参数取自
`config/reality/base.yaml` 或 `config/simulation/base.yaml`，先验地图路径由
`prior_pcd_file` launch 参数注入：

```bash
cd ~/guganav
scripts/simulation.sh nav rmul_2025 prior_pcd_file:=/abs/path/rmul_2025.pcd
```

### 输入与输出

| 话题 / TF | 类型 | 方向 | 说明 |
|-----------|------|------|------|
| `registered_scan` | `sensor_msgs/PointCloud2` | 订阅 | Point-LIO 输出的 `odom` 系注册点云，累积后作为 GICP 源点云 |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅 | RViz 手动初始位姿，仅在自动重定位失败或需要人工干预时使用 |
| TF `base_frame <- lidar_frame` | `tf2` | 启动时查询一次 | 把雷达里程计系的先验地图换算到 `odom` 系；查不到时每秒重试 |
| TF `odom_frame <- base_frame` | `tf2` | 每轮查询 | 越界检查时计算机器人在地图中的水平位置 |
| TF `robot_base_frame <- current_scan_frame_id` | `tf2` | 收到 `initialpose` 时 | 把 RViz 给的机器人位姿换算成 `map -> odom` |
| TF `map -> odom` | `tf2` | 发布（20 Hz） | 唯一的输出，即定位修正量 |

先验 PCD 由 Point-LIO 一类算法在雷达里程计系下建图得到，节点在初始化阶段用
`base_frame <- lidar_frame` 的安装变换把它换算到 `odom` 系，因此项目其余节点看到的
始终是与底盘对齐的 `odom` 系。

### 参数

参数在两个 `guga_bringup` 配置文件中分别标定，下面是实车 `config/reality/base.yaml`
的取值：

```yaml
small_gicp_relocalization:
  ros__parameters:
    # 点云预处理与 GICP
    num_threads: 8                    # OpenMP 线程数
    num_neighbors: 20                 # 估计协方差时的近邻数
    global_leaf_size: 0.1             # 先验地图降采样体素边长 (m)
    registered_leaf_size: 0.05        # 累积扫描降采样体素边长 (m)
    max_dist_sq: 6.0                  # 对应点距离平方上限 (m^2)
    max_iterations: 100               # 配准质量不达标时的迭代上限
    max_roll_pitch_step: 0.00002      # 单次迭代 roll/pitch 增量的模长上限 (rad), 0 关闭
    max_tz_step: 0.00001              # 单次迭代 tz 增量的大小上限 (m), 0 关闭
    # 质量门与基准 tf
    error_max: 0.6                    # 迭代脉冲触发上界: 上一帧 RMSE 超此值(或内点率不达标)则本次拉满 max_iterations
    good_error_max: 0.4               # 可升级为基准 tf 的 RMSE 上限
    good_inlier_ratio: 0.99           # 可升级为基准 tf 的内点率下限
    baseline_translation_tolerance: 4.0  # 惯性约束: 候选相对基准 tf 的平移偏差上限 (m)
    map_boundary_margin: 1.0          # 位姿超出先验地图外接框的允许余量 (m)
    # 坐标系
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_footprint"
    robot_base_frame: "gimbal_yaw"
    lidar_frame: "front_mid360"
    init_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
    # 先验地图路径通常由 launch 的 prior_pcd_file 参数注入, 不必写在这里
    prior_pcd_file: ""
```

仿真配置除线程数与阈值外基本一致（`num_threads: 16`、
`baseline_translation_tolerance: 6.0`、`map_boundary_margin: 0.5`）。`error_max`、
`good_error_max`、`good_inlier_ratio` 需要按日志里打印的 RMSE 与内点率标定。

### 重定位逻辑

- 启动阶段加载 `prior_pcd_file`，用 `base_frame <- lidar_frame` 把先验地图换算到
  `odom` 系并计算地图外接框；随后按 `global_leaf_size` 降采样、估计协方差、建 KD 树。
  TF 没就绪时每秒重试一次，不阻塞地图就绪之后的流程。
- 2 Hz（500 ms）把累积的 `registered_scan` 作为源点云，按 `registered_leaf_size`
  降采样后做 GICP。`StepLimitOptimizer` 限制每次迭代的 roll/pitch 增量
  （`max_roll_pitch_step`）和 tz 增量（`max_tz_step`），避免配准一次把位姿掀翻。
- 质量门：RMSE = `sqrt(error / num_inliers)`，内点率 = `num_inliers / 源点数`。只有
  `RMSE < good_error_max`、内点率 `> good_inlier_ratio` 且未越出地图外接框
  （`map_boundary_margin`）的结果，才能升级为新的基准 tf。
- 惯性约束：候选相对当前基准 tf 的平移偏差超过 `baseline_translation_tolerance` 时
  直接丢弃并回到基准，保证只在基准附近做平移搜索；只有通过质量门的候选才刷新基准。
- 迭代脉冲：上一帧 RMSE ≤ `error_max` 且内点率达标时，本次只迭代 10 次（快路径）；
  任一不达标就把迭代上限拉满到 `max_iterations` 重新收敛。
- 手动定位：还没有基准时，RViz 的 `initialpose` 直接作为基准（绕过质量门）；已有基准
  时只作下一轮 GICP 的种子，仍需通过质量门才能升级为基准。
- 对外发布：无论候选是否被采纳，20 Hz 广播的始终是基准 tf，避免 `map -> odom` 每轮
  抖动。发布前会把 z 置零并只保留 yaw（`regulateRegistration`），即输出是平面对齐的
  2D 位姿。

---

下方内容来自上游 small_gicp_relocalization，保留用于算法背景与参数含义参考；其中涉及
独立仓库目录、旧构建命令或旧 launch 文件的步骤不适用于本工作空间。

> 上游仓库: [SMBU-PolarBear-Robotics-Team/small_gicp_relocalization](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/LihanChen2004/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

### Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

### Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

### Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```
