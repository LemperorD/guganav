# pb2025_robot_description

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

SMBU PolarBear Team robot description package for RoboMaster 2025.

深圳北理莫斯科大学北极熊战队 - RoboMaster 2025 赛季通用机器人关节描述包。

## 1. Overview

本功能包含深圳北理莫斯科大学北极熊战队用于 RoboMaster 赛事的机器人关节描述文件。它将读取机器人描述文件并转换为 TF 和 joint_states ，以便与其他 ROS2 节点配合使用。

本项目使用 [xmacro](https://github.com/gezp/xmacro) 格式描述机器人关节信息，可以更灵活的组合已有模型。

当前机器人描述文件基于 [rmua19_standard_robot](https://github.com/robomaster-oss/rmoss_gz_resources/tree/humble/resource/models/rmua19_standard_robot) 进行二次编辑，加入了工业相机和激光雷达等传感器。

- [buaa2026_sentry_robot](./resource/xmacro/buaa2026_sentry_robot.sdf.xmacro)

    仿真与实车统一加载这一份机器人描述，搭载云台相机 industrial_camera 和 Livox mid360，
    其中相机与 gimbal_pitch 轴固连，mid360 倾斜侧放与 chassis 固连。双方使用同一组
    `base_footprint -> chassis -> front_mid360` TF，避免仿真和实车 lidar→odom
    安装关系各自维护后失配。

    仿真安装尺寸以 `buaa2026_sentry_robot` 为准：`base_footprint -> chassis`
    高度为 0.123 m；`chassis -> front_mid360` 的 xyz 为 `[0.225, 0, 0.107]` m，
    rpy 为 `[-10, 0, -90]` 度。合成后的雷达高度为 0.230 m。
    云台 yaw/pitch 安装高度分别为 0.026/0.355 m。

    MID360 模型中的点云原点与 `front_mid360` 重合，IMU 位置对应实车
    `extrinsic_T = [-0.011, -0.02329, 0.04412]`（LiDAR 到 IMU）。
    仿真 Point-LIO 的重力方向也与实车配置一致，按仿真 IMU 单位归一到
    9.81 m/s²；`acc_norm` 仍为 9.81，实车为 1.0，不能直接复制该单位参数。

    修改模型后需重启 Gazebo，并重新生成配套 PCD 和栅格地图。
    旧安装配置生成的地图不会自动转换。重新安装资源与启动建图：

    ```bash
    colcon build --symlink-install --packages-select pb2025_robot_description guga_bringup
    source install/setup.bash
    scripts/simulation.sh map rmul_2026
    ```

    ![sentry](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/sentry_description.1sf3yc69kr.webp)

    ![frames](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames.5xaq4wriyy.webp)

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Create Workspace

```bash
sudo apt install git-lfs
pip install vcstool2
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git
```

```bash
vcs import --recursive < dependencies.repos
```

```bash
pip install xmacro
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install
```

### 2.4 Running

#### Option1: 在 RViz 中可视化机器人

```bash
ros2 launch pb2025_robot_description robot_description_launch.py
```

#### Option2: Python API

通过 Python API，在 launch file 中解析 XMacro 文件，生成 URDF 和 SDF 文件 (Recommend)：

> [!TIP]
>
> [robot_state_publisher](https://github.com/ros/robot_state_publisher) 需要传入 urdf 格式的机器人描述文件
>
> Gazebo 仿真器 spawn robot 时，需要传入 sdf / urdf 格式的机器人描述文件

感谢前辈的开源工具 [xmacro](https://github.com/gezp/xmacro) 和 [sdformat_tools](https://github.com/gezp/sdformat_tools) ，这里简述 XMacro 转 URDF 和 SDF 的示例，用于在 launch file 中生成 URDF 和 SDF 文件。

```python
from xmacro.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator

xmacro = XMLMacro4sdf()
xmacro.set_xml_file(robot_xmacro_path)

# Generate SDF from xmacro
xmacro.generate()
robot_xml = xmacro.to_string()

# Generate URDF from SDF
urdf_generator = UrdfGenerator()
urdf_generator.parse_from_sdf_string(robot_xml)
robot_urdf_xml = urdf_generator.to_string()
```

#### Option3: 命令行

通过命令行直接转换输出 SDF 文件（Not Recommend）:

```bash
source install/setup.bash

xmacro4sdf src/pb2025_robot_description/resource/xmacro/simulation_robot.sdf.xmacro > src/pb2025_robot_description/resource/xmacro/simulation_robot.sdf
```

## 3. Subscribed Topics

None.

## 4. Published Topics

- `robot_description (std_msgs/msg/String)`

    机器人描述文件（字符串形式）。

- `joint_states (sensor_msgs/msg/JointState)`

    如果命令行中未给出 URDF，则此节点将侦听 `robot_description` 话题以获取要发布的 URDF。一旦收到一次，该节点将开始将关节状态发布到 `joint_states` 话题。

- `any_topic (sensor_msgs/msg/JointState)`

    如果 `sources_list` 参数不为空（请参阅下面的参数），则将订阅此参数中的每个命名话题以进行联合状态更新。不要将默认的 `joint_states` 话题添加到此列表中，因为它最终会陷入无限循环。

- `tf, tf_static (tf2_msgs/msg/TFMessage)`

    机器人关节坐标系信息。

## 5. Launch Arguments

- `use_sim_time` (bool, default: False)

    是否使用仿真时间。

- `robot_name` (str, default: "simulation_robot")

    机器人 XMacro 描述文件的**名字（无需后缀）**。描述文件应位于 `package://pb2025_robot_description/resource/xmacro` 目录下。

- `robot_xmacro_file` (str, default: "[simulation_robot.sdf.xmacro](./resource/xmacro/simulation_robot.sdf.xmacro)")

    机器人 XMacro 描述文件的**绝对路径**。本参数的优先级高于 `robot_name`，即若设置了 `robot_xmacro_file`，则 `robot_name` 参数无效。若未设置 `robot_xmacro_file`，则使用 `robot_name` 参数并自动补全路径作为 `robot_xmacro_file` 的值。

- `params_file` (str, default: [robot_description.yaml](./params/robot_description.yaml))

- `rviz_config_file` (str, default: [visualize_robot.rviz](./rviz/visualize_robot.rviz))

    RViz 配置文件路径。

- `use_rviz` (bool, default: True)

    是否启动 RViz 可视化界面。

- `use_respawn` (bool, default: False)

    是否在节点退出时尝试重启节点。

- `log_level` (str, default: "info")

    日志级别。
