# guga_evaluate

机器人自身导航评测工具，不依赖裁判系统。它订阅控制指令、里程计、路径、目标点和可选的
Gazebo 真值，实时显示速度响应、路径误差和系统状态。默认不保存任何数据，需要时可显式
开启 CSV 和 JSON 记录。

## 快速启动

先构建并加载工作空间：

```bash
cd ~/guganav
colcon build --symlink-install --packages-select guga_evaluate
source install/setup.bash
```

导航系统已经运行后，另开终端启动评测：

```bash
# 实车
scripts/evaluate.sh reality

# 仿真，默认命名空间与 simulation_launch.py 一致
scripts/evaluate.sh simulation

# 显示并保存数据
scripts/evaluate.sh reality --save

# 自定义输出目录和命名空间
scripts/evaluate.sh simulation --save --output output/my_run \
  --namespace red_standard_robot1

# 无图形界面运行（例如 SSH 或自动测试）
scripts/evaluate.sh simulation --no-gui
```

实时界面默认开启，显示以下四个区域：

- 指令线速度与实际线速度
- 指令角速度与实际角速度
- 横向路径误差与目标距离
- 当前误差、RMSE、话题频率、目标状态和记录状态

关闭界面不会停止评测节点；在启动评测的终端按 `Ctrl-C` 可以完整结束。
使用 `--save` 时，节点每 5 秒刷新一次 `summary.json`，异常退出时也能保留最近汇总。

也可以直接运行 launch：

```bash
ros2 launch guga_evaluate evaluate.launch.py \
  namespace:=red_standard_robot1 \
  mode:=simulation \
  use_sim_time:=true \
  use_ground_truth:=true \
  save_data:=true \
  show_visualization:=true \
  output_dir:=/tmp/guga_eval
```

## 默认订阅

| 数据 | 相对话题 |
| --- | --- |
| 速度指令 | `cmd_vel` |
| 导航里程计 | `odometry` |
| 局部路径 | `local_plan` |
| 全局路径 | `plan` |
| MPC 预测路径 | `predicted_plan` |
| 目标点 | `goal_pose` |
| Gazebo 真值 | `chassis_odometry_gt`，仅仿真模式 |
| 实时评测指标 | `evaluation_metrics`，由评测节点发布 |

话题均为相对名称，可以通过 `namespace` 或 `config/evaluate.yaml` 适配。

## 保存选项和输出文件

脚本默认传入 `save_data:=false`，不会创建输出目录或文件。添加 `--save` 后才会生成：

| 文件 | 内容 |
| --- | --- |
| `metadata.json` | 模式、主机、ROS 版本和全部参数 |
| `state.csv` | 位姿、里程计速度、位姿差分速度 |
| `command.csv` | `cmd_vel`、加速度和 jerk |
| `tracking.csv` | 路径误差、目标误差、速度误差、真值误差 |
| `plans.csv` | 路径长度、点数和曲率 |
| `events.csv` | 目标接收和目标到达事件 |
| `summary.json` | 频率、延迟、RMSE、P50/P95/P99 等汇总 |

速度跟踪默认使用连续位姿差分得到的车体系速度，而不是盲目信任 `Odometry.twist`。
路径跟踪优先使用 `local_plan`，没有局部路径时回退到 `plan`。不同坐标系之间通过 TF
变换；TF 不可用时跳过该帧指标并给出一次警告。重复发布的相同目标会自动去重，
不会被当成多次独立任务。

## 指标边界

- 实车没有外部真值时，速度和路径跟踪结果包含定位误差。
- 仿真真值与导航里程计必须使用相同 frame 才会计算定位误差。
- 当前工具不计算障碍物净空和碰撞指标，后续需要接入 costmap 与 Gazebo 碰撞事件。
