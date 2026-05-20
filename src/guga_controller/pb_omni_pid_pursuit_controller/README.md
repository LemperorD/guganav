# pb_omni_pid_pursuit_controller

全向 PID 纯追踪控制器，Nav2 Controller 插件。接收全局路径，输出 `cmd_vel`（`vx, vy, wz`），支持 holonomic 全向底盘。

## 架构

```
OmniPidPursuitControllerNode (Nav2 插件入口)
  ├─ PathHandler          — 路径 TF 变换 + 裁切
  ├─ PID (move_pid_)      — 平移距离 PID
  ├─ PID (heading_pid_)   — 旋转角度 PID
  ├─ geometry_utils       — 圆-线段交点、曲率半径、累积距离、路径插值
  └─ visualization_helper — carrot 点、曲率标记可视化
```

## 管道

```
computeVelocityCommands (20Hz)
  │
  ├─ transformPath        — global_plan → TF → 裁切已走过 → local_plan
  ├─ computeLookahead     — 速度缩放前视距离 → 找 carrot 点
  ├─ computeVelocity      — PID(lin_dist, 0) → lin_vel, PID(angle, 0) → angular_vel
  ├─ applyVelocityLimits  — 曲率减速 + 接近减速
  ├─ checkCollision       — 采样 N 点转全局坐标 → costmap 查代价
  └─ assembleCmdVel       — 组装 vx/vy/wz
```

## ChassisMode

| 值 | 模式 | enable_rotation | 行为 |
|----|------|-----------------|------|
| 1 | CHASSIS_FOLLOWED | true | vx/vy 沿路径方向分解 + wz 旋转跟踪 |
| 2 | LITTLE_TES | false | vx/vy 沿 carrot 方向分解 + wz=0，纯平移 |
| 3 | GO_HOME | — | 已定义，暂未处理 |

## 参数

### PID

| 参数 | 默认 | 说明 |
|------|------|------|
| `translation_kp/ki/kd` | 3.0/0.1/0.3 | 平移 PID |
| `rotation_kp/ki/kd` | 3.0/0.1/0.3 | 旋转 PID |
| `min_max_sum_error` | 1.0 | 积分限幅（当前未生效，已知 bug） |

### 前视

| 参数 | 默认 | 说明 |
|------|------|------|
| `lookahead_dist` | 0.3 | 固定前视距离 (m) |
| `use_velocity_scaled_lookahead_dist` | true | 速度缩放前视 |
| `min_lookahead_dist` | 0.2 | 最小前视 (m) |
| `max_lookahead_dist` | 1.0 | 最大前视 (m) |
| `lookahead_time` | 1.0 | 前视时间 (s) |
| `use_interpolation` | true | 圆-线段交点插值 carrot |
| `use_rotate_to_heading` | true | 终点原地旋转对齐朝向 |
| `use_rotate_to_heading_treshold` | 0.1 | 旋转阈值 (rad) |

### 限速

| 参数 | 默认 | 说明 |
|------|------|------|
| `v_linear_min/max` | -3.0/3.0 | 线速度范围 (m/s) |
| `v_angular_min/max` | -3.0/3.0 | 角速度范围 (rad/s) |
| `min_approach_linear_velocity` | 0.05 | 接近终点最低速 (m/s) |
| `approach_velocity_scaling_dist` | 0.6 | 接近减速距离 (m) |

### 曲率

| 参数 | 默认 | 说明 |
|------|------|------|
| `curvature_min` | 0.4 | 低曲率阈值，低于此不减速 |
| `curvature_max` | 0.7 | 高曲率阈值，高于此大幅减速 |
| `reduction_ratio_at_high_curvature` | 0.5 | 高曲率速度降比 |
| `curvature_forward_dist` | 0.7 | 曲率前向采样距离 (m) |
| `curvature_backward_dist` | 0.3 | 曲率后向采样距离 (m) |
| `max_velocity_scaling_factor_rate` | 0.9 | 曲率减速率限制 |

### 其他

| 参数 | 默认 | 说明 |
|------|------|------|
| `transform_tolerance` | 0.1 | TF 变换超时 (s) |
| `collision_sample_points` | 10 | 碰撞检测采样点数 |
| `max_robot_pose_search_dist` | — | 路径搜索范围（自动 = costmap 半宽） |

## 注册

```xml
<class type="pb_omni_pid_pursuit_controller::OmniPidPursuitControllerNode"
       base_class_type="nav2_core::Controller">
```

Nav2 YAML 配置：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitControllerNode"
```

## 测试

```bash
colcon test --packages-select pb_omni_pid_pursuit_controller
```

## 发布的话题

| Topic | 类型 | 说明 |
|-------|------|------|
| `local_plan` | `nav_msgs/Path` | 变换后的局部路径（采样 10 点） |
| `lookahead_point` | `geometry_msgs/PointStamped` | carrot 前视点 |
| `curvature_points_marker_array` | `visualization_msgs/MarkerArray` | 曲率计算点 |

## 订阅的话题

| Topic | 类型 | 说明 |
|-------|------|------|
| `chassis_mode` | `std_msgs/UInt8` | 底盘模式切换 |
