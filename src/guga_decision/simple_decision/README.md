# simple_decision

规则决策节点，RMUL/RMUC 哨兵中负责根据裁判系统数据选择行为模式（补给/攻击/默认）并发布导航目标。

## 架构

```
ROS2 消息 → Convert*     → EnvironmentContext → buildSnapshot       → Decision     → executeAction → ROS2 消息
 (订阅)      (转换层)        (环境聚合层)          (快照)             (决策层)       (执行层)        (发布)
                                  ↕
                            updateTracking
```

- **转换层** (`adapter/`) — ROS msg → 领域类型，纯函数
- **环境层** (`core/environment_context`) — 聚合多个消息为统一状态，自带线程安全
- **决策层** (`core/decision`) — 纯计算，读取快照返回动作，无副作用、无依赖
- **节点层** (`node/`) — ROS2 接线，参数声明，组装并驱动上述三层

## 状态机

```
DEFAULT ──(hp低/弹药空)──→ SUPPLY ──(hp恢复)──→ DEFAULT
   │                          │
   └──(敌人出现)──→ ATTACK ←──┘
```

| 状态 | 底盘模式 | goal |
|------|---------|------|
| DEFAULT | CHASSIS_FOLLOWED | 默认坐标 |
| ATTACK | CHASSIS_FOLLOWED / LITTLE_TES (受击) | 最近敌人 |
| SUPPLY | CHASSIS_FOLLOWED | 补给点 |

## 门控

比赛开始前不发决策，start_delay_sec 后解除：

```
无 RS  → 不发
无 GS  → 不发 (require_game_running=true)
比赛未开始 → 不发
delay 内 → 不发
delay 后 → 发
```

## 构建

```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash
cd ~/guganav
colcon build --packages-select simple_decision
```

## 运行

```bash
source ~/guganav/install/setup.bash
ros2 launch simple_decision simple_decision.py
```

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `frame_id` | map | TF 父帧 |
| `base_frame_id` | base_footprint | TF 子帧 |
| `tick_hz` | 20.0 | 决策频率 |
| `hp_survival_enter` | 120 | HP 低于此值进补给 |
| `hp_survival_exit` | 300 | HP 高于此值出补给 |
| `ammo_min` | 0 | 弹药等于此值进补给 |
| `start_delay_sec` | 5.0 | 比赛开始后延迟 |
| `combat_max_distance` | 8.0 | 敌人检测最大距离 |
| `default_x/y/yaw` | 2.0/0.5/0.0 | 默认导航点 |
| `supply_x/y/yaw` | 0.0/0.0/0.0 | 补给点 |
| `default_goal_hz` | 2.0 | 默认态 goal 发布频率 |
| `supply_goal_hz` | 2.0 | 补给态 goal 发布频率 |
| `attack_goal_hz` | 10.0 | 攻击态 goal 发布频率 |

## 依赖

| 包 | 用途 |
|---|------|
| `rclcpp` | ROS2 客户端库 |
| `rclcpp_components` | 组件节点注册 |
| `tf2_ros` | TF 坐标变换 |
| `geometry_msgs` | Pose/Point 消息 |
| `guga_interfaces` | 裁判系统/装甲/目标消息 |

## 测试

详见 [test/README.md](test/README.md)

```bash
# 构建 + 单元测试 + 集成测试 + 覆盖率报告
scripts/test/test_simple_decision_coverage.sh
```
