# sentry_comm

读写 `Sentry2Communicator` 共享内存快照的通信模块：按固定频率读取快照，按固定频率写入控制命令，并提供写入导航速度与桥接裁判系统数据的两个节点。

## 结构

| 文件 | 内容 |
| --- | --- |
| `include/sentry_comm/snapshot_reader.hpp` | `SnapshotReader`：固定频率读取、缓存最近一份快照 |
| `include/sentry_comm/command_writer.hpp` | `CommandWriter`：固定频率写入最近一次提交的命令 |
| `include/sentry_comm/nav_command_node.hpp`、`src/nav_command_node.cpp` | `nav_command_node`：把 `cmd_vel`、过洞模式与里程计写入共享内存 |
| `include/sentry_comm/referee_bridge_node.hpp`、`src/referee_bridge_node.cpp`、`src/referee_mapping.*` | `referee_bridge_node`：发布裁判系统话题与云台关节状态 |
| `launch/nav_command.launch.py`、`launch/referee_bridge.launch.py` | 两个节点的启动文件 |
| `src/snapshot_reader.cpp`、`src/command_writer.cpp` | 两个类的实现 |
| `test/snapshot_reader_test.cpp`、`test/command_writer_test.cpp` | 读取与写入的用例 |

客户端代码与共享定义来自 `Sentry2Communicator/ipc_client`，构建时直接编译其中的 `IpcClient.cpp` 与 `SerialCommClient.cpp`，安装时把这套头文件复制到 `include/sentry_comm/s2c_client`。构建机需要保留 `Sentry2Communicator`：默认先找与工作区并列的目录，再找工作区内部的目录，两个位置都不对时用 `-DSENTRY2_COMMUNICATOR_DIR=/path/to/Sentry2Communicator` 指定。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select sentry_comm
colcon test --packages-select sentry_comm
colcon test-result --verbose
```

测试会在本进程内创建共享内存服务端，读取用例验证发布后的值被取到，写入用例注册 `VisWrite` 与 `NavWrite` 的接收回调并验证命令送达，不需要串口硬件。

## 读取

```cpp
#include "sentry_comm/snapshot_reader.hpp"

sentry_comm::SnapshotReader reader({"/s2c_serial_comm", 100.0});
reader.start();

s2c::serial_comm::AllInfo snapshot{};
if (reader.latestSnapshot(snapshot)) {
  const double mcu_time = snapshot.server_info.last_mcu_receive_time;
}
```

`Config::read_rate_hz` 默认 100 Hz，构造时校验为有限正数。读取线程按固定周期取最近一份快照，两次读取之间发布的新帧会被跳过。`latestSnapshot` 返回的是副本，取到 `true` 之后不再受读取线程影响。

判据与读取方式：

- `latestSnapshot` 返回 `false` 表示服务端不可用或还没有成功读取过任何快照。
- 电控与 IMU 是否在线，用 `server_info.last_mcu_receive_time` 与 `last_imu_receive_time` 判断，启动前后者为 0。
- `Stats::attempts` 是读取次数，`successes` 与 `failures` 分别是成功与失败次数，`last_success_time` 是最近一次成功的 `steady_clock` 秒数。

## 写入

```cpp
#include "sentry_comm/command_writer.hpp"

sentry_comm::NavCommandWriter writer({"/s2c_serial_comm", 100.0});

s2c::serial_comm::NavWrite command{};
command.serial.deform_chassis.displacement_target_velocity_x = 0.3f;
command.serial.deform_chassis.displacement_target_velocity_y = 0.0f;
writer.setCommand(command);
writer.start();

writer.update([](s2c::serial_comm::NavWrite& write) {
  write.serial.deform_chassis.displacement_target_velocity_x = 0.1f;
});
```

`VisCommandWriter` 写火控与四个轴，`NavCommandWriter` 写变形与底盘位移，两个类型各自只占用自己的通道。`Config::write_rate_hz` 默认 100 Hz。

写入语义：

- 只有在 `setCommand` 或 `update` 之后才会发送；没有命令时线程空转，不会把零初始化的命令写给电控。
- 线程每个周期发送最近一次提交的完整命令。`setCommand` 覆盖整份命令，`update` 在锁内修改其中的字段。
- 命令会一直重发，直到被下一次提交替换。调用方进程退出后，服务端仍会发送最后一次命令，退出前需要主动写回安全命令。
- 火控脉冲由调用方写 1 再写 0，写入周期决定脉冲的最短宽度；100 Hz 下最短约 10 毫秒。
- 同一个通道只应由一个进程写。两个进程同时写同一个通道时，每次提交都会覆盖对方的内容。
- 只改一个字段时，先用 `setCommand` 提交一份完整命令（通常以快照的 `send_data` 为基底），之后用 `update` 修改字段。

判据与写入方式：

- `Stats::attempts` 只统计有命令时的发送次数，`successes` 与 `failures` 分别统计服务端可用与不可用时的结果。
- `command()` 返回最近一次提交的命令副本，`hasCommand()` 表示是否提交过命令。
- 重复 `start()` 抛出 `std::logic_error`，`stop()` 可以重复调用。

## 发送节点

`nav_command_node` 把导航栈现在交给旧串口驱动的输入写到新的共享内存，订阅与字段对应关系如下。

| 输入 | 字段 | 单位 |
| --- | --- | --- |
| `cmd_vel` 的 `linear.x` | `displacement_target_velocity_x` | 米每秒 |
| `cmd_vel` 的 `linear.y` | `displacement_target_velocity_y` | 米每秒 |
| `cmd_vel` 的 `angular.z` | `displacement_target_velocity_yaw` | 弧度每秒 |
| `/decision/pitch_mode` | `deform_control` | 位，当前直接搬字节值 |
| `odometry` 的位姿 | `nav_to_vision.current_position` 与 `orientation_quaternion` | 米、四元数 |
| `odometry` 的速度 | `nav_to_vision.current_velocity` | 米每秒，按姿态转到 odom 系 |

节点每 20 毫秒组装一份完整的 `NavWrite` 并提交，写入线程按 100 Hz 重发。超过 `command_timeout_sec`（默认 0.5 秒）没有新的 `cmd_vel`，平移与 yaw 速度清零，`deform_control` 保留。`nav_to_vision` 的预测点与建议大 yaw 暂时保持 0，等视觉侧确认需要哪些字段再补。

```bash
ros2 launch sentry_comm nav_command.launch.py
```

参数：`shm_name`（默认 `/s2c_serial_comm`）、`write_rate_hz`（100）、`command_timeout_sec`（0.5）、`cmd_vel_topic`（`cmd_vel`）、`odometry_topic`（`odometry`）、`pitch_mode_topic`（`/decision/pitch_mode`）。

旧 `serial_driver` 还负责把 `/referee/*`、`/serial/Yaw` 等接收数据发布成 ROS 话题。切到新链路时要停掉旧驱动，接收侧需要另行迁移，否则决策层拿不到裁判数据。

## 裁判系统桥接

`referee_bridge_node` 从快照的 `mcu_data` 里取裁判系统数据段，按 50 Hz 发布决策层订阅的三个话题。

| 话题 | 来源数据段 | 填充字段 |
| --- | --- | --- |
| `/referee/robot_status` | 0x0201、0x0202、0x0206 | 机器人编号与等级、血量、冷却值、热量上限、17mm 热量、装甲编号、扣血原因、`is_hp_deduced` |
| `/referee/game_status` | 0x0001 | 比赛阶段、阶段剩余时间 |
| `/referee/rfid_status` | 0x0209 | 24 个增益点位 |
| `/serial/gimbal_joint_state` | `yaw_big_position` | 云台到底盘的单个转角 |

```bash
ros2 launch sentry_comm referee_bridge.launch.py
```

`is_hp_deduced` 按相邻两次采样比较血量，与 `fake_msg_source` 的做法一致，只在血量下降的那一次发布为真。电控数据超过 `stale_timeout_sec`（默认 1 秒）没有更新时停止发布，与旧驱动没有新帧就不发布的行为一致。

电控反馈里没有 0x0208 数据段，`projectile_allowance_17mm` 与 `remaining_gold_coin` 只能填 0。决策层的发弹量判断依赖这个字段，需要电控把 0x0208 加进反馈包后才能恢复。

`/serial/gimbal_joint_state` 只发布一个关节，默认名 `gimbal_yaw_odom_joint`，角度取大 yaw 的 `yaw_big_position`。`invert_joint_angle` 控制是否取反，默认 `true` 沿用旧驱动对 yaw 差的处理；新模型的关节轴与编码器正方向一致时设为 `false`。关节名用 `joint_name` 参数修改。

旧驱动发布的 `/serial/Yaw`、`/serial/EnemyPos` 没有迁移，导航栈里没有消费者。

## 切换启动

`communication_launch.py` 增加了 `use_sentry_comm` 参数，默认 `True` 启动 `sentry_comm` 的两个节点、不启动 `serial_driver`；设为 `False` 回退到旧驱动。

```bash
ros2 launch guga_bringup reality_launch.py use_sentry_comm:=true
```

前提是 `s2c_serial_comm_server` 已经在运行，否则两个节点会持续报告写入失败，决策层也收不到裁判数据。

## 下一步

电控补上 0x0208 数据段，确认新模型里云台到底盘关节的名字与符号。
