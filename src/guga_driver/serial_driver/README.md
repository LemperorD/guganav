> 测试使用的命令
```
ros2 topic pub -r 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

# serial_driver

ROS 2 与 MCU 之间的 BR 二进制串口协议驱动。该包负责串口收发、BR
帧解析、ROS topic 桥接、裁判系统数据发布和部分云台 TF 广播。

详细的线协议和 payload 字段布局见
[BR_PROTOCOL.md](../../../docs/BR_PROTOCOL.md)。

## Architecture

```text
ROS topics
    |
    v
SerialDriverNode
  ROS 编解码、topic、timer、TF
    |
    v
RosToSerialBridge<T> / SerialToRosBridge<T>
  ROS message <-> MotionPayload (单向，分别处理 PC→MCU 和 MCU→PC)
    |
    v
SerialDriverMain
  fd、BR 组帧/拆帧、CRC8、快照、重连
    |
    v
Linux serial device <-> MCU
```

主要文件：

| 文件 | 职责 |
|---|---|
| `br_protocol_types.hpp` | 帧常量、payload 类型、上下行及裁判字段偏移 |
| `serial_driver_main.hpp/.cpp` | 串口生命周期、收发、帧解析、快照和重连 |
| `ros_serial_bridge.hpp` | 单个 ROS topic 与运动 payload 的桥接模板 |
| `serial_driver_node.hpp/.cpp` | ROS 节点、编解码、裁判数据和 TF |

## Data Flow

PC 到 MCU：

```text
/cmd_vel: geometry_msgs/Twist
  -> encodeTwist()
  -> MotionPayload[17]
  -> sendDataFrame()
  -> BR frame[22]
  -> MCU
```

MCU 到 PC：

```text
MCU motion frame
  -> receiveDataFrameSnapshot()
  +-> decodeYaw()      -> /serial/Yaw
  +-> decodeEnemyPos() -> /serial/EnemyPos

MCU referee frame
  -> takeRefereeFrameSnapshot()
  +-> /referee/robot_status
  +-> /referee/game_status
  +-> /referee/rfid_status
```

`receiveDataFrameSnapshot()` 返回加锁复制的最新运动帧。两个接收 bridge
（Yaw、EnemyPos）各自以 200 Hz 读取该快照，因此在没有新运动帧时会重复发布
最后一帧。

`takeRefereeFrameSnapshot()` 原子完成“检查新帧、复制、清除 ready 标志”，
每个裁判帧最多消费一次。

## ROS Interface

订阅：

| Topic | Type | 作用 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 生成并下发 17 字节运动 payload |

发布：

| Topic | Type | 来源 |
|---|---|---|
| `/serial/Yaw` | `std_msgs/msg/Float32` | 运动帧 yaw 差值 |
| `/serial/EnemyPos` | `geometry_msgs/msg/Point` | 运动帧敌方坐标 |
| `/referee/robot_status` | `guga_interfaces/msg/RobotStatus` | 裁判帧机器人状态 |
| `/referee/game_status` | `guga_interfaces/msg/GameStatus` | 裁判帧比赛阶段 |
| `/referee/rfid_status` | `guga_interfaces/msg/RfidStatus` | 裁判帧 RFID 位图 |

TF：每 30 ms 查询 `odom -> base_footprint`，成功后广播
`base_footprint -> gimbal_yaw_vision`。

## Parameters

| 参数 | 代码默认值 | 说明 |
|---|---:|---|
| `port_name` | `/dev/ttyACM0` | 串口设备；空字符串启用自动探测 |
| `baud_rate` | `115200` | 支持 9600 到 230400 的预设波特率 |
| `vel_trans_scale` | `40.0` | PC 速度写入 payload 前的缩放系数（仅 vx/vy，wz 不缩放） |
| `lidar_ip` | `192.168.1.2` | MID360 连通性检测地址 |
| `lidar_port` | `56360` | MID360 连通性检测端口 |

启动文件默认加载 `config/serial_driver.yaml`，其中的值可以覆盖代码默认值。

## Build And Run

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select serial_driver
source install/setup.bash
ros2 launch serial_driver communication.launch.py
```

指定配置：

```bash
ros2 launch serial_driver communication.launch.py \
  communication_cfg_dir:=/absolute/path/to/serial_driver.yaml
```

## Hardware Smoke Test

实机测试至少检查：

1. 发布 `/cmd_vel` 后 MCU 能正确收到速度（vx/vy 已缩放、wz 未缩放）。
2. `/serial/Yaw`、`/serial/EnemyPos` 字段与 MCU 一致。
3. 每个裁判帧只触发一次 `/referee/*` 更新。
4. 拔插串口或重启 MCU 后，驱动能在重连周期内恢复。
5. 高频发送时没有 `TX failed`，CRC 错误帧不会更新 payload 快照。
6. 雷达不可达时，单次连通性检测在约 500 ms 内返回。

常用观察命令：

```bash
ros2 topic hz /serial/Yaw
ros2 topic echo /serial/EnemyPos
ros2 topic echo /referee/robot_status
```

## Current Limitations

- 两个 MCU 到 ROS bridge 使用独立线程轮询同一运动帧快照，不能保证两个
  topic 总是来自同一次发布周期。
- 运动帧没有 new-frame 标志，MCU 停止发送后仍会重复发布最后一帧。
- `SerialToRosBridge` 接收线程以 `rclcpp::ok()` 为退出条件，独立卸载 component
  时需要进一步验证析构行为。
- MCU yaw 的角度单位需要实机确认：`decodeYaw` 按弧度语义直接读取，未做
  度/弧度换算；当前 `encodeTwist` 不消费 `yaw_diff`（云台→底盘旋转变换
  `transformVelocityToChassis` 暂未接入）。
- 当前包只有 lint 配置，尚无协议单元测试、PTY 集成测试或 fake transport 测试。
