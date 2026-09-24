# 无实车 UI 标签测试（假串口 / 假下位机）

在**没有实车**（没有串口下位机、没有裁判系统）的条件下，验证 UI 标签能否实时更新，
例如 HUD 里的 `Robot Yaw`（云台-底盘 yaw 角差）和 `Match Status`。

原理：用 Python 自带的 pty 创建一个虚拟串口，按 `docs/BR_PROTOCOL.md` 的 BR 帧协议
持续向 `serial_driver` 喂运动帧和裁判帧。`serial_driver` 收到帧后照常解析并写入
`guga_shm` 的 YAW 槽位，UI 就能像连着实车一样读到数据。整个过程不改动任何项目代码。

## 涉及文件

| 文件 | 说明 |
| ---- | ---- |
| `scripts/fake_serial_driver/fake_mcu.py` | 假下位机：创建虚拟串口并持续发送 BR 帧（运动帧 + 裁判帧） |
| `scripts/fake_serial_driver/shm_yaw_probe.py` | 诊断工具：直接读 YAW 槽，判断是写端还是 UI 端的问题 |

## 前置条件

1. 代码已编译且**进程已重启**（最容易被忽略）：

   ```bash
   cd ~/guganav
   source install/setup.bash
   colcon build --symlink-install --packages-select serial_driver guga_ui_pangolin
   ```

2. 确认当前 `serial_driver` 源码里 `decodeYaw()` 包含 YAW 槽写入
   （搜索 `shm_writer_yaw_.write`，缺失时 UI 永远收不到 yaw 数据）。

## 操作步骤（三个终端，顺序不能乱）

**终端 1：启动假下位机**

```bash
cd ~/guganav
python3 scripts/fake_serial_driver/fake_mcu.py
```

记下输出的虚拟串口路径，例如：

```
[fake_mcu] 虚拟串口: /dev/pts/5
[fake_mcu] serial_driver 启动参数: --ros-args -p port_name:=/dev/pts/5
```

**终端 2：启动 serial_driver**（必须先于 UI）

```bash
cd ~/guganav
source install/setup.bash
ros2 run serial_driver serial_driver_node_exe --ros-args -p port_name:=/dev/pts/5
```

把 `/dev/pts/5` 换成终端 1 实际输出的路径（每次运行可能不同）。

**终端 3：启动 UI**

```bash
cd ~/guganav
source install/setup.bash
./install/guga_ui_pangolin/lib/guga_ui_pangolin/guga_ui guga_shm
```

> UI 只在启动时打开一次共享内存：如果 UI 先于 serial_driver 启动，
> 它会以“空数据”模式运行且不会重试，需要重启 UI。

## 预期效果

- **HUD.Robot Yaw**：从 `--` 变为数值，并在约 `-0.80 ~ +0.80` 弧度之间缓慢往复变化。
- **Match Status**：显示 `4`（假下位机每秒发送一帧裁判帧，比赛阶段=4）。

## 工作原理简记

- 帧格式：`0x42 0x52 | CMD | LEN | PAYLOAD | CRC8`
  - 运动帧 `0xCD`，payload 17 字节：`[0:4]` yaw 角差（**度**）、`[4:8]` 敌方 x、`[8:12]` 敌方 y。
  - 裁判帧 `0xD1`，payload 13 字节：`[0]` 高 4 位为比赛阶段。
- CRC8 为 CRC-8/MAXIM（反射多项式 0x8C、初值 0xFF），脚本里的查表与 `serial_driver` 完全一致。
- `decodeYaw()` 目前把上报值按**度**处理并转成弧度（`/180*π`），所以脚本发度数、UI 显示弧度。

## 常见问题排查

| 现象 | 排查方向 |
| ---- | -------- |
| 标签一直是 `--` | YAW 槽从未写入。先跑 `python3 scripts/fake_serial_driver/shm_yaw_probe.py`：若 `seq` 恒为 0，说明 serial_driver 没写 YAW 槽（旧二进制 / 未重启 / 源码缺 `shm_writer_yaw_.write`）；若 `seq` 在涨，说明 UI 进程是旧的，重启 UI。 |
| 标签一直是 `0.00` | serial_driver 在写槽但值是 0：多半是串口没收到有效帧（CRC 失败、端口连错、假下位机没先启动）。用 `ros2 topic echo /serial/Yaw` 确认串口侧数值。 |
| 数值不变化 | 假下位机发送的 yaw 是固定值，或 gimbal 确实没动；脚本默认是正弦往复，若改过确认公式/频率。 |
| `ros2 run` 报找不到包 | 没有 `source install/setup.bash`，或该包未编译。 |
| CRC8 failed 刷屏 | 假下位机帧校验不过：确认脚本未被改动、连接的是脚本输出的同一端口。 |

## 自定义发送内容

想改 yaw 波形/频率，直接编辑 `scripts/fake_serial_driver/fake_mcu.py`：

- yaw 波形：`main()` 里的 `0.8 * math.sin(0.5 * t)`（幅度 0.8 rad，角频率 0.5 rad/s）；
- 发送频率：`time.sleep(0.02)`（当前 50Hz）；
- 裁判阶段：`referee(progress=4)` 的参数。

改完重跑终端 1 的脚本即可，serial_driver 3 秒无数据后会自动重连。
