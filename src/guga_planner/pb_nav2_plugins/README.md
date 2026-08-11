# pb_nav2_plugins

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Overview

`pb_nav2_plugins` 是一个用于扩展 `Navigation2`（Nav2）框架的插件库，当前提供提供了一些额外的行为逻辑和代价地图层以增强机器人在导航过程中灵活性。

## 2. Plugins

### 2.1 Behaviors

#### 2.1.1 BackUpFreeSpace

`BackUpFreeSpace` 插件用于在导航恢复过程中寻找并执行一段全向脱困运动，主要功能：

1. **全向搜索**：在请求距离内搜索可行方向，并优先选择角度裕量更大、累计代价更低的方向。
2. **驶出初始高代价区**：当机器人起始格为 `253/254` 时，允许穿过与起点连续且代价不增加的高代价区域。
3. **验证安全终点**：沿选定方向继续搜索，直到找到通过完整 footprint 碰撞检查的最近终点。
4. **恢复严格检查**：离开初始高代价区后，若再次遇到 `253/254`，立即拒绝该方向。

**Parameters:**

- `max_radius`: 单次恢复动作允许请求的最大移动距离（default：1.0 m）。
- `max_escape_distance`: 起始位置处于 253/254 高代价区时，允许驶出该区域的最大距离（default：0.5 m）。
- `escape_clearance`: 首次到达 `<253` 区域后继续移动的安全余量（default：0.1 m）。
- `service_name`: 获取代价图的服务名称（default："local_costmap/get_costmap"）。
- `visualize`: 是否启用可视化功能。启用后会在 RViz 中显示自由空间和目标位置（default：false）。

当机器人起始格为 `253/254` 时，插件只允许穿过与起点连续且代价不增加的高代价前缀，
并要求在 `max_escape_distance` 内进入 `<253` 区域。实际移动距离会限制到选出的安全终点，
不会继续执行原始请求中超出安全终点的部分。

**Example:**

```yaml
behavior_server:
ros__parameters:
   use_sim_time: true
   local_costmap_topic: local_costmap/costmap_raw
   global_costmap_topic: global_costmap/costmap_raw
   local_footprint_topic: local_costmap/published_footprint
   global_footprint_topic: global_costmap/published_footprint
   cycle_frequency: 10.0
   behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
   spin:
      plugin: "nav2_behaviors/Spin"
   backup:
      plugin: "pb_nav2_behaviors/BackUpFreeSpace"
   drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
   wait:
      plugin: "nav2_behaviors/Wait"
   assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
   local_frame: odom
   global_frame: map
   robot_base_frame: base_footprint
   transform_tolerance: 0.1
   simulate_ahead_time: 2.0
   max_rotational_vel: 1.0
   min_rotational_vel: 0.4
   rotational_acc_lim: 3.2
   # params for pb_nav2_behaviors/BackUpFreeSpace
   max_radius: 3.5
   max_escape_distance: 0.5
   escape_clearance: 0.1
   service_name: "global_costmap/get_costmap"
   visualize: True
```

### 2.2 Layers

#### 2.2.1 IntensityVoxelLayer

`IntensityVoxelLayer` 是一个用于处理点云数据中障碍物强度信息的代价地图层。它可以根据点云数据中的强度值来标记障碍物，并将这些障碍物信息添加到代价地图中，本插件推荐配合 [terrain_analysis](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis) 功能包使用。

**Parameters:**

Common to [costmap-plugins/voxel.html](https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html)。

主要区别在于添加了 `min_obstacle_intensity` 和 `max_obstacle_intensity` 参数，注意此参数使用域在 `obstacle_layer` 下，而非 `observation_sources` 下。

**Example:**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.2
      plugins: ["static_layer", "intensity_voxel_layer", "inflation_layer"]
      intensity_voxel_layer:
        plugin: pb_nav2_costmap_2d::IntensityVoxelLayer
        enabled: true
        track_unknown_space: true
        footprint_clearing_enabled: true
        publish_voxel_map: false
        combination_method: 1
        mark_threshold: 0
        origin_z: 0.0
        unknown_threshold: 5
        z_resolution: 0.05
        z_voxels: 16
        min_obstacle_height: 0.0
        max_obstacle_height: 2.0
        min_obstacle_intensity: 0.1
        max_obstacle_intensity: 2.0
        observation_sources: terrain_map
        terrain_map:
          data_type: PointCloud2
          topic: /terrain_map
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.2
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.7
      always_send_full_costmap: False
```

## Acknowledgements

The Initial Developer of some parts of the repository (`BackUpFreeSpace`, `IntensityVoxelLayer`, `IntensityVoxelLayer`), which are copied from, derived from, or
inspired by @PolarisXQ [SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master/nav2_plugins/behavior_ext_plugins), @ros-navigation [navigation2](https://github.com/ros-navigation).
All Rights Reserved.
