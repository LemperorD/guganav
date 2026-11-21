# MID-360 IMU 读取与六面采集

这个程序直接从 MID-360 读取板载 IMU，不经过 ROS 话题，也不经过串口。数据链路是
Livox SDK2 的 IMU 回调：雷达通过网口把数据发到主机，SDK 解析后回调给本程序。

SDK 给的是原始量：加速度单位 g，角速度单位 rad/s。程序把加速度换算成 m/s^2，
输出的 CSV 与 `scripts/imu_calib` 的输入格式一致，可以直接交给求解器。

## 网络配置

雷达与主机的地址由配置文件决定，默认使用仓库里的实车配置：

```
src/guga_bringup/config/reality/mid360_user_config.json
```

其中主机 IP 是 `192.168.11.50`，雷达 IP 是 `192.168.11.3`，端口从 56100 到
56501。主机 IP 必须配置在连接雷达的那块网卡上，否则 SDK 启动时会报
`bind failed`。用别的配置文件时通过 `--config` 指定。

### 网络配对

第一次连接雷达时，先给网口配好地址。脚本会选中有载波的有线网口，按
NetworkManager 配置静态地址（重启后保留），然后 ping 雷达：

```bash
sudo scripts/imu_lidar/setup_net.sh
```

雷达不可达时，脚本会扫描 `192.168.11.0/24` 并列出有响应的地址，用来判断雷达
实际 IP 是否真的是 `192.168.11.3`。移除配置用 `--remove`，指定网口用
`--iface enp108s0`。

也可以手动配置：

```bash
sudo nmcli connection modify "Wired connection 1" ipv4.method manual \
    ipv4.addresses 192.168.11.50/24 ipv4.never-default yes
sudo nmcli connection up "Wired connection 1"
ping -c 3 192.168.11.3
```

网口名字用 `ip -br link` 查看。台式机或笔记本的板载网口一般是有线网口，
无线网口名以 `wl` 开头。

## 编译

仓库里已经带了各架构的 SDK2 预编译库，编译时按当前架构选择：

```bash
bash scripts/imu_lidar/build.sh
```

产物是 `scripts/imu_lidar/imu_lidar`，已在 `.gitignore` 中忽略。程序通过
rpath 指向仓库内的 `Livox-SDK2/lib/<架构>`，移动仓库位置后需要重新编译。

## 用法

先确认能收到数据：

```bash
scripts/imu_lidar/imu_lidar
```

默认进入监视模式，每 0.2 s 打印一组加速度与角速度，按 Ctrl+C 退出。静止时
加速度模长应接近 9.807 m/s^2。

做六面采集：

```bash
scripts/imu_lidar/imu_lidar --six-face --duration 5 \
    --out /tmp/imu_six_face.csv
```

程序依次提示六个朝向，每个朝向按回车后先等角速度平稳，再采集指定时长。
完成后调用求解器：

```bash
python3 scripts/imu_calib/imu_calib.py solve --csv /tmp/imu_six_face.csv \
    --out /tmp/imu_calibration.json
```

没有雷达时可以验证整条链路：

```bash
python3 scripts/imu_lidar/test_sim.py
```

它会用 `--simulate` 跑完六个朝向的采集，再调用求解器，检查参数能否恢复到
合成真值。

## 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--config` | 仓库实车配置 | 雷达网络配置文件 |
| `--six-face` | 关 | 交互式六面采集 |
| `--out` | `imu_six_face.csv` | 采集输出路径 |
| `--duration` | 5 | 每个朝向的采集时长，单位秒 |
| `--still-timeout` | 10 | 等待静止的最长时间，单位秒 |
| `--still-gyro-std` | 0.02 | 判定静止的角速度标准差，单位 rad/s |
| `--simulate` | 关 | 用合成数据跑完整流程，不需要雷达 |

## 输出格式

CSV 列为 `t,ax,ay,az,wx,wy,wz,face`：

- `t` 是相对第一帧的秒数；
- `ax` 到 `az` 单位 m/s^2；
- `wx` 到 `wz` 单位 rad/s；
- `face` 取 `+x` 到 `-z` 六种。

## 常见问题

`bind failed`：配置文件里的主机 IP 不在任何网卡上。先把雷达所在网卡配置成
`192.168.11.50/24`，或者把配置文件里的地址改成网卡实际地址。

10 s 内没有收到 IMU 数据：检查雷达是否上电、网线是否接在配置的网卡上、雷达 IP
是否为 `192.168.11.3`。程序会在超时后给出同样的提示。

端口被占用：同一台机器上重复启动两个读取程序会抢占 56100 到 56501 端口，
先关掉旧进程，或者确认 ROS 驱动没有在跑。

## 已知限制

- 六面法的假设与限制见 `scripts/imu_calib/README.md`，本程序只负责采集。
- 程序不做雷达与 IMU 的外参和时间偏移标定。
- IMU 数据的时间戳取自主机收到回调的时刻，不是雷达内部的硬件时间戳。六面
  标定只用均值，这个差别不影响结果。
