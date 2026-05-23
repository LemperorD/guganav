# pb_omni_pid_pursuit_controller

全向 PID 纯追踪控制器，Nav2 Controller 插件。接收全局路径，输出 `cmd_vel`（`vx, vy, wz`），支持 holonomic 全向底盘。

## 架构分层

```
Layer 4: Controller   OmniPidPursuitControllerNode  — Nav2 插件入口，编排下层
Layer 3: Adapter      PathHandler                    — 路径 TF 变换 + 裁切
Layer 2: Core         PID, geometry_utils, visualise — 纯计算，无副作用
Layer 1: Types        ControllerConfig, ControllerState, ChassisMode — 数据结构
```

所有符号均在 `pb_omni_pid_pursuit_controller` 命名空间内（`visualization_helper`、`geometry_utils` 为独立子命名空间）。

## 管道

```
computeVelocityCommands (目标 20Hz)
  │
  ├─ transformPath        — global_plan → TF → 裁切已走过 → local_plan
  ├─ computeLookahead     — 速度缩放前视距离 → 找 carrot 点
  ├─ computeVelocity      — PID(lin_dist, 0) → lin_vel, PID(angle, 0) → angular_vel
  ├─ applyVelocityLimits  — 曲率减速（含速率限制） + 接近减速
  ├─ checkCollision       — 采样 N 点转全局坐标 → costmap 查代价
  └─ assembleCmdVel       — 组装 vx/vy/wz
```

## ChassisMode

| 值  | 模式             | enable_rotation | 行为                                    |
| --- | ---------------- | --------------- | --------------------------------------- |
| 1   | CHASSIS_FOLLOWED | true            | vx/vy 沿路径方向分解 + wz 旋转跟踪      |
| 2   | LITTLE_TES       | false           | vx/vy 沿 carrot 方向分解 + wz=0，纯平移 |
| 3   | GO_HOME          | —               | 已定义，暂未处理                        |

## 参数

### PID

| 参数                   | 默认        | 说明                          |
| ---------------------- | ----------- | ----------------------------- |
| `translation_kp/ki/kd` | 3.0/0.1/0.3 | 平移 PID                      |
| `rotation_kp/ki/kd`    | 3.0/0.1/0.3 | 旋转 PID                      |
| `enable_rotation`      | true        | 启用旋转跟踪（false 时 wz=0） |
| `min_max_sum_error`    | 1.0         | 积分限幅                      |

### 前视

| 参数                                 | 默认 | 说明                   |
| ------------------------------------ | ---- | ---------------------- |
| `lookahead_dist`                     | 0.3  | 固定前视距离 (m)       |
| `use_velocity_scaled_lookahead_dist` | true | 速度缩放前视           |
| `min_lookahead_dist`                 | 0.2  | 最小前视 (m)           |
| `max_lookahead_dist`                 | 1.0  | 最大前视 (m)           |
| `lookahead_time`                     | 1.0  | 前视时间 (s)           |
| `use_interpolation`                  | true | 圆-线段交点插值 carrot |
| `use_rotate_to_heading`              | true | 终点原地旋转对齐朝向   |
| `use_rotate_to_heading_threshold`    | 0.1  | 旋转阈值 (rad)         |

### 限速

| 参数                             | 默认     | 说明                   |
| -------------------------------- | -------- | ---------------------- |
| `v_linear_min/max`               | -3.0/3.0 | 线速度范围 (m/s)       |
| `v_angular_min/max`              | -3.0/3.0 | 角速度范围 (rad/s)     |
| `min_approach_linear_velocity`   | 0.05     | 接近终点保底速度 (m/s) |
| `approach_velocity_scaling_dist` | 0.6      | 接近减速距离 (m)       |

### 曲率

| 参数                                | 默认 | 说明                         |
| ----------------------------------- | ---- | ---------------------------- |
| `curvature_min`                     | 0.4  | 低曲率阈值，低于此不减速     |
| `curvature_max`                     | 0.7  | 高曲率阈值，高于此用最低比率 |
| `reduction_ratio_at_high_curvature` | 0.5  | 高曲率速度降比               |
| `curvature_forward_dist`            | 0.7  | 曲率前向采样距离 (m)         |
| `curvature_backward_dist`           | 0.3  | 曲率后向采样距离 (m)         |
| `max_velocity_scaling_factor_rate`  | 0.9  | 曲率减速平滑率 (/s)          |

### 其他

| 参数                         | 默认         | 说明                                |
| ---------------------------- | ------------ | ----------------------------------- |
| `transform_tolerance`        | 0.5          | TF 变换超时 (s)，传递给 PathHandler |
| `collision_sample_points`    | 10           | 碰撞检测采样点数                    |
| `max_robot_pose_search_dist` | costmap 半宽 | 路径搜索范围，自动推算              |

## 依赖

```
pb_omni_pid_pursuit_controller
├── nav2_core          — Controller 基类
├── nav2_costmap_2d    — 碰撞检测
├── nav2_util          — euclidean_distance, calculate_path_length
├── tf2_ros            — TF 坐标变换
├── rclcpp_lifecycle   — 生命周期节点 (仅在 Layer 4)
├── geometry_msgs      — 消息类型
├── nav_msgs           — Path 消息
├── visualization_msgs — Marker 可视化
└── std_msgs           — UInt8 (ChassisMode 订阅)
```

## 测试

```bash
colcon build --packages-select pb_omni_pid_pursuit_controller
./test_pid && ./test_geometry_utils && ./test_visualise \
  && ./test_pathhandler && ./test_approach_scaling && ./test_types
```

共 22 个测试用例，覆盖 PID 六项、geometry_utils 八项、visualise 一项、PathHandler 一项、approach scaling 三项、types 三项。

## 性能

控制器代码本身 0.5–0.6ms/帧（约 0.1ms PID + 0.4ms transformPath + 0.07ms limits），远在 20Hz（50ms）预算之内。若 `/cmd_vel` 实际频率偏低，瓶颈在 Nav2 框架层（costmap 更新频率、executor 线程模型），建议检查 `nav2_params.yaml` 中 `controller_frequency` 和 `local_costmap/update_frequency` 配置。

## 发布的话题

| Topic                           | 类型                             | 说明             |
| ------------------------------- | -------------------------------- | ---------------- |
| `local_plan`                    | `nav_msgs/Path`                  | 变换后的局部路径 |
| `lookahead_point`               | `geometry_msgs/PointStamped`     | carrot 前视点    |
| `curvature_points_marker_array` | `visualization_msgs/MarkerArray` | 曲率计算点       |

## 订阅的话题

| Topic          | 类型             | 说明         |
| -------------- | ---------------- | ------------ |
| `chassis_mode` | `std_msgs/UInt8` | 底盘模式切换 |
