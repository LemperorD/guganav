# pl_cmp — Point-LIO 两个版本的对比工具

用于比较 `refactoring_pointlio` 分支上的 point_lio 与 `origin/main` 上原版的行为，
判断重构是否改变了估计结果、发布行为与资源占用。

数据全部是合成的，不依赖雷达硬件：

- 场景：8 m × 8 m × 3 m 封闭房间（地面、天花板、四面墙）
- 运动：半径 2 m 圆周（0.4 m/s、0.2 rad/s）叠加 1 Hz 垂直起伏，姿态只有 yaw
- 雷达：`livox_ros_driver2/msg/CustomMsg`，10 Hz，2 万点/帧，点写在各自采样时刻的雷达系下，
  因此天然带帧内运动畸变；`tag = 0x10`、`line = i % 4`，与 `avia_handler` 的过滤条件一致
- IMU：200 Hz，加速度以 g 为单位（`acc_norm = 1.0` 的约定），带固定零偏与噪声
- 时长 21 s：雷达 191 帧、IMU 4201 条、378 万点，bag 约 74 MB

## 用法

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # 需要 livox_ros_driver2 的 Python 绑定

# 1. 生成合成 bag（写入 $PL_CMP_DIR，默认 <ws>/.pl_cmp_bench）
python3 scripts/pl_cmp/gen_bag.py

# 2. 跑被测版本；参数是要 source 的 install/setup.bash，最后一个生效
bash scripts/pl_cmp/run_case.sh after_fix install/setup.bash

# 3. 对比（标签对应 out/<label>）
python3 scripts/pl_cmp/compare_csv.py original after_fix
```

跑原版需要单独取一份 `origin/main` 的 point_lio 并编译：

```bash
mkdir -p /tmp/main_ws/src
git archive origin/main src/guga_localization/point_lio \
  | tar -x -C /tmp/main_ws/src --strip-components=2
cd /tmp/main_ws && colcon build --packages-select point_lio --symlink-install
```

环境变量：`PL_CMP_DIR` 指定临时目录（默认 `<ws>/.pl_cmp_bench`），
`MONITOR_TOPICS` 选择监视的话题（`odom`、`odom+cloud+path`，默认后者），
`DRAIN_LIMIT` 为回放结束后等待节点处理完积压帧的上限秒数（默认 600）。

## 脚本

| 文件 | 作用 |
|------|------|
| `gen_bag.py` | 生成合成输入 bag |
| `monitor.py` | 订阅输出话题只记时间戳与规模，不保存点云（替代 rosbag2 录制） |
| `run_case.sh` | 启动监视器与节点、回放 bag、采样 CPU 与峰值内存、清理进程 |
| `extract_bag.py` | 从输出 bag 用 SQL 取出里程计（避免读取巨大的点云 blob） |
| `compare_csv.py` | 两版轨迹互相比较，并对齐到合成真值 |

注意：不能用 `rosbag2` 录制输出。重构版曾以约 500 Hz 重复发布整幅点云，
30 秒就能写出 15 GB，所以监视器只记录 `points`、`pose_count` 这类规模指标。

## 结果（`results/` 中的数据）

修复前（`refactoring_pointlio` 的 `ae7d4b9` 之前）与修复后分别测同一份输入：

| 指标 | main 原版 | 重构版（修复前） | 重构版（修复后） |
|------|-----------|------------------|------------------|
| 处理帧数 | 190 | 191 | 191 |
| node CPU | 2.85 s | 612 s（600 s 仍未结束） | 5.81 s |
| 峰值内存 | 93 MB | 368 MB | 69 MB |
| `cloud_registered` 频率 | 10 Hz | 约 500 Hz | 10 Hz |
| `path` 位姿数 | 190（每帧一个） | 134217 | 191 |
| 相对合成真值位置 RMSE | 0.0208 m | — | 0.0208 m |

两版逐一对比（共同帧 190 帧）：位置差 RMSE 2.7 mm、最大 24 mm、终点差 3.3 mm，
yaw 差 RMSE 0.0106°。估计结果等价，差异在浮点量级。

真值对齐后有约 11.7° 的常量偏置与 0.42 m 平移，来自滤波器初始化时的重力对齐与
IMU 零偏，不是漂移：去掉偏置后 yaw 误差 RMSE 为 0.017°（原版）与 0.045°（重构版）。

## 已知前提

- `run_case.sh` 不能用 `set -u`（ROS 的 setup.bash 会读未定义的 `AMENT_TRACE_SETUP_FILES`），
  也不能用 `/usr/bin/time` 包装节点（杀掉包装进程不会终止真正的节点，残留节点会污染下一次录制）。
- ROS 需要写 `~/.ros`，若运行在受限文件系统下，把 `ROS_HOME` 指到可写目录，
  否则 `ros2 bag` 会段错误。
- 这里提交的脚本相对当初实测时的版本只改了路径解析（改为按脚本位置推导工作区），
  其余逻辑一致，但未用调整后的版本重跑验证。
