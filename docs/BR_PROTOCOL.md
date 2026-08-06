# BR 串口通信协议

> 上位机 (PC) ↔ 下位机 (MCU) 之间基于 RS232/USB-TTL 的自定义二进制帧协议。
> BR = Beihang Robotics。

---

## 1. 物理层

| 项目 | 值 |
|------|-----|
| 波特率 | 115200（可配：9600 / 19200 / 38400 / 57600 / 115200 / 230400） |
| 数据位 | 8 |
| 校验位 | 无 (N) |
| 停止位 | 1 |
| 硬件流控 | 无 |
| 软件流控 | 无 |
| 读取模式 | 非阻塞，VMIN=0, VTIME=1（最多等待 0.1 秒） |
| 上位机轮询 | ~1kHz 固定周期线程 |

---

## 2. 帧格式

```
┌────────┬────────┬────────┬────────┬──────────────────┬────────┐
│  SOF0  │  SOF1  │  CMD   │  LEN   │     PAYLOAD      │  CRC8  │
│ 0x42   │ 0x52   │  1B    │  1B    │      n 字节       │  1B    │
└────────┴────────┴────────┴────────┴──────────────────┴────────┘
```

| 字段 | 大小 | 说明 |
|------|------|------|
| SOF | 2 字节 | 帧头 `0x42 0x52` = ASCII `"BR"` (Beihang Robotics) |
| CMD | 1 字节 | 命令码，决定 payload 语义 |
| LEN | 1 字节 | payload 长度（不含 SOF/CMD/LEN/CRC8） |
| PAYLOAD | n 字节 | 可变长数据，n = LEN |
| CRC8 | 1 字节 | 校验 SOF + CMD + LEN + PAYLOAD |

- 最小帧长：**5 字节**（n=0）
- 当前驱动允许的最大帧长：**128 字节**
- 接收缓存：256 字节线性字节缓存
- 单次回调最多处理：10 帧

### CRC8 参数

```
多项式:  0x31 (x^8 + x^5 + x^4 + 1)
初始值:  0xFF
方式:    查表驱动 (256 字节查找表)
```

---

## 3. 命令码

| CMD | 名称 | 方向 | Payload 长度 |
|-----|------|------|:---:|
| `0xCD` | 运动控制帧 | PC ↔ MCU 双向 | 26 字节 |
| `0xD1` | 裁判系统帧 | MCU → PC | 13 字节 |

---

## 4. 运动控制帧 (`0xCD`) — 26 字节

### 4.1 PC → MCU（上位机下发）

对应函数: `encodeTwist(geometry_msgs::msg::Twist) → MotionPayload`

```
Offset  Size  字段           类型        说明
────────────────────────────────────────────────────────
[0]     1     chassis_mode   uint8_t     底盘模式
[1-4]   4     angle_init     float LE    初始角度
[5]     1     flag           uint8_t     标志位（固定 1）
[6-9]   4     vx_Y           float LE    底盘系 x 线速度 × vel_trans_scale
[10-13] 4     vy_Y           float LE    底盘系 y 线速度 × vel_trans_scale
[14-17] 4     vx             float LE    云台系 x 线速度 × vel_trans_scale
[18-21] 4     vy             float LE    云台系 y 线速度 × vel_trans_scale
[22-25] 4     -wz            float LE    角速度 z 取反
```

**注意**: 速度值经过 `vel_trans_scale_`（默认 40.0）线性缩放，由 `float` 转 `uint8_t[4]` 小端序写入。
`vx_Y` / `vy_Y` 经过 `yaw_diff` 从云台系旋转变换到底盘系。

### 4.2 MCU → PC（下位机上传）

下位机通过同一 26 字节缓冲区回传多种数据，上位机按需读取不同字节段：

| 方法 | 话题 | 读取范围 | 类型 | 含义 |
|------|------|---------|------|------|
| `decodeYaw` | `/serial/Yaw` | `[7-10]` | float LE | 云台 yaw 角度差 |
| `decodeTESspeed` | `/serial/TES_speed` | `[3-6]` | float LE | TES 角速度 z |
| `decodeEnemyPos` | `/serial/EnemyPos` | `[11-12]` | int16_t LE | 敌方 x 坐标 |
| | | `[13-14]` | int16_t LE | 敌方 y 坐标 |

---

## 5. 裁判系统帧 (`0xD1`) — 13 字节（MCU → PC）

对应函数: `publishRefereeData()`，20ms 周期发布。

```
Offset  Size  字段              类型          说明
─────────────────────────────────────────────────────────────
[0]     1     game_progress     uint8_t       高4位：比赛阶段
              (reserved)                      低4位：保留
[1-2]   2     current_hp        uint16_t LE   当前血量
[3-4]   2     ammo_17mm         uint16_t LE   17mm 弹丸余量
[5-6]   2     heat_17mm_1       uint16_t LE   1号枪管热量
[7-8]   2     reserved          —             保留
[9-12]  4     rfid_status       uint32_t LE   RFID 状态位掩码
```

### 5.1 RFID 位映射（裁判系统协议 V1.7.0 `0x0209`）

| Bit | 含义 |
|:---:|------|
| 0 | 基地增益点 |
| 1 | 中央高地增益点 |
| 2 | 敌方中央高地增益点 |
| 3 | 我方梯形高地增益点 |
| 4 | 敌方梯形高地增益点 |
| 5 | 我方飞坡前增益点 |
| 6 | 我方飞坡后增益点 |
| 7 | 敌方飞坡前增益点 |
| 8 | 敌方飞坡后增益点 |
| 9 | 我方中央高地低位增益点 |
| 10 | 我方中央高地高位增益点 |
| 11 | 敌方中央高地低位增益点 |
| 12 | 敌方中央高地高位增益点 |
| 13 | 我方高速公路低位增益点 |
| 14 | 我方高速公路高位增益点 |
| 15 | 敌方高速公路低位增益点 |
| 16 | 敌方高速公路高位增益点 |
| 17 | 我方前哨站增益点 |
| 18 | 我方哨站增益点 |
| 19 | 我方补给区（不可兑换） |
| 20 | 我方补给区（可兑换） |
| 21 | 我方大资源岛 |
| 22 | 敌方大资源岛 |
| 23 | 中心增益点 |

### 5.2 发布话题

| 话题 | 消息类型 | 字段 |
|------|---------|------|
| `/referee/robot_status` | `guga_interfaces::msg::RobotStatus` | current_hp, projectile_allowance_17mm, shooter_17mm_1_barrel_heat |
| `/referee/game_status` | `guga_interfaces::msg::GameStatus` | game_progress |
| `/referee/rfid_status` | `guga_interfaces::msg::RfidStatus` | 24 个 bool 字段（各增益点状态） |

---

## 6. 话题桥接架构

```
ROS2 话题                        RosMcuBridge                        串口帧
──────────                      ────────────                        ──────
/cmd_vel         ──encodeTwist──▶ bridge_twist_pc_     ──sendDataFrame──▶ 0xCD → MCU
/serial/Yaw      ◀─decodeYaw────  bridge_yaw_mcu_      ◀─receiveDataFrameSnapshot─ 0xCD ← MCU
/serial/TES_speed◀─decodeTESspeed bridge_tes_speed_mcu_◀─receiveDataFrameSnapshot─ 0xCD ← MCU
/serial/EnemyPos ◀─decodeEnemyPos bridge_enemy_pos_mcu_◀─receiveDataFrameSnapshot─ 0xCD ← MCU
/referee/*  ×3   ◀─publishRefereeData (timer 20ms)     ◀─takeRefereeFrameSnapshot 0xD1 ← MCU
/chassis_mode    ──subscription──▶ chassis_mode_ 成员变量                   (本地)
```

---

## 7. 断线重连

- **超时**: 3 秒无数据触发重连
- **重连间隔**: 最少 3 秒间隔
- **重连流程**: `close(fd_)` → 重置缓冲区 → `openSerialPort()` 或自动探测 → 重置时间戳
- **自动探测**: 扫描 `/dev/ttyUSB*` 和 `/dev/ttyACM*`（`std::filesystem` 实现）

---

## 8. 关键源文件

| 文件 | 内容 |
|------|------|
| `br_protocol_types.hpp` | 帧常量、payload 类型和字段偏移 |
| `serial_driver_main.hpp` | 驱动参数、CRC8 表和类声明 |
| `serial_driver_main.cpp` | 帧解析、收发、重连实现 |
| `serial_driver_node.hpp` | ROS 话题桥接和节点声明 |
| `serial_driver_node.cpp` | 编解码函数、裁判数据解析、tf 广播 |
| `ros_serial_bridge.hpp` | 通用话题↔串口桥接模板 |
