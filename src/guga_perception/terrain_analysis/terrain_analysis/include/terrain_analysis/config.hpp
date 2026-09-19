#pragma once

/**
 * @brief 管线两半各自读取的参数。
 *
 * 拆成两个结构体是为了让"哪一半读哪些参数"在类型上就可见——两半的分工不同
 * （前半段负责接收与累积，后半段负责估计与判定），参数混在一个结构里时看不出
 * 边界。两半都不持有配置：参数由调用方（节点或测量工具）在调用点注入。
 *
 * 距离单位为米，时间单位为秒。
 */

/**
 * @brief 管线前半段（PersistentVoxelMap）读取的参数。
 *
 * 前半段负责"接收并累积观测"：融合叶尺寸、时间衰减、输入高度带与接收半径。
 * 与 PerFrameHeightConfig 分开，是为了让每一半读了哪些参数在类型上就可见——原先
 * 一个 TerrainConfig 里 16 个参数混在一起，看不出哪些属于哪一半。
 *
 * 距离单位为米，时间单位为秒。
 */
struct PersistentVoxelConfig {
  /** @brief 融合叶尺寸（水平，x/y）。 */
  double scan_voxel_size = 0.1;
  /** @brief 融合叶尺寸（垂直，z）。
   *
   *  垂直方向必须比水平方向细：每个叶只保留最新观测的那一个点，若地面点与
   *  矮物体点落进同一个叶，地面点每帧都被观测到，会把物体点顶掉（叶宽 0.2 m
   *  时实测 6 cm 矮台阶的输出点数归零）。实测：水平 0.1 / 垂直 0.05 相比两者
   *  都取 0.05，单帧耗时降到约 56%，而矮台阶输出点数不再下降。 */
  double scan_voxel_size_z = 0.05;
  /** @brief 历史体素点的衰减时间。 */
  double decay_time = 2.0;
  /** @brief 在该距离内不执行时间衰减。 */
  double no_decay_distance = 4.0;

  /** @brief 接收带的下沿。**雷达系**：相对雷达高度（`point.z − lidar_z`）。
   *
   *  两处共用同一判据（`PersistentVoxelMap::insideReceiveBand`）：ingest
   * 的裁剪、 rebuild 的保留。地面候选的地板不用它——那是 **odom 绝对**的
   * ground_floor_z， 属于 PerFrameHeightConfig。 */
  double min_relative_z = -1.5;
  /** @brief 接收带的上沿。**雷达系**，与下沿同一个判据。 */
  double max_relative_z = 0.2;
  /** @brief 接收带随水平距离放宽的比例（带宽 = 该比例 × 水平距离）。 */
  double distance_ratio_z = 0.2;

  /** @brief 地形体素边长（同时也是接收半径的基准）。 */
  double terrain_voxel_size = 1.0;
};

/**
 * @brief 管线后半段（PerFrameHeightMap）读取的参数。
 *
 * 后半段负责"由累积观测估计地面并判定障碍"：地面估计方式、地面带死区、障碍输出
 * 高度带、平面网格分辨率。
 *
 * `min_relative_z` 与 PersistentVoxelConfig 同名，这是有意的：它是同一个 ROS
 * 参数
 * （`minRelZ`），但两半的用途不同——前半段用它定义接收带的下沿，后半段用它挡掉
 * 远低于雷达的穿透点（障碍输出的地板）。节点从同一个参数同时填入两处。
 *
 * 距离单位为米。
 */
struct PerFrameHeightConfig {
  /** @brief 是否使用分位数估计地面高度，否则使用最小值。 */
  bool use_sorting = true;
  /** @brief 地面高度分位数。 */
  double quantile_z = 0.25;
  /** @brief 是否将负高度差转换为绝对值。 */
  bool consider_drop = false;
  /** @brief 是否限制地面估计相对最低点的抬升量。 */
  bool limit_ground_lift = false;
  /** @brief 地面估计允许的最大抬升量。 */
  double max_ground_lift = 0.15;

  /** @brief planar voxel 的最小有效点数。 */
  int min_block_point_num = 10;
  /** @brief 障碍输出下界。**地面系**：距局部地面小于该值的点不作为障碍输出
   *  （判据 `PerFrameHeightMap::insideOutputBand`）。
   *
   *  作为"地面带"的死区使用，吸收地面高度估计的误差：估计值偏低时，真实地面点
   *  会算出几厘米的正高度，若下界为 0，这些点会被当作低矮障碍标记出去。
   *  实车取值 0.04 m。
   *
   *  注意与清除用途的分工：清除用的回波**不应**受本下界约束（地面回波正是"射线
   *  路径为空"的证据），见设计文档 R4 对 marking / clearing 两份点云的区分。
   *  `consider_drop` 打开时高度取绝对值，本下界对凹坑同样成立。 */
  double min_obstacle_height = 0.04;
  /** @brief 障碍输出上界。**地面系**：距**局部地面**达到该值的点不作为障碍
   *  （车辆可从其下方通过，或高于车体不构成碰撞威胁）。
   *
   *  按实车车体高度 + 100 mm 安全间隙设定（车高 520 mm → 0.62）。因车高随车
   *  而异（后续有第二台车），故为 ROS 参数而非编译期常量。
   *
   *  这是障碍输出**唯一**的高度上界，且只作用于**地面上方**的点（判据里的 h 带
   *  符号）；`consider_drop` 打开时凹坑按 |h| 输出，其 intensity 可以超过本值。
   *  曾另有一条 `height < vehicle_height` 的
   *  截断，它与本值重叠且更严（0.52 < 0.62），会把车高与净空之间那一带的点
   *  一并丢掉，而那个高度上的悬空结构车是过不去的——该截断已移除，参数一并
   *  删除。换车只需重设本值。 */
  double ceiling_clearance = 0.62;

  /** @brief 障碍输出的地板（相对雷达）：低于该高度的点不输出，用于挡掉穿透点。
   *
   *  与 PersistentVoxelConfig 的同名字段来自同一个 ROS
   * 参数，用途见本结构体注释。
   */
  double min_relative_z = -1.5;

  /** @brief 地面候选的绝对高度地板（odom z）：低于该值的点不参与地面估计。
   *
   *  用绝对量而非"相对雷达"：地面在 odom 中大体水平，且该阈值不应随雷达
   *  俯仰/上下抖动而移动。仅由 estimateTerrainGround 使用，且是该阶段**唯一**
   *  的候选筛选（原有的净空上界已移除，见该函数注释）。
   *
   *  取值须贴近实际地面 z：odom 原点与 base_footprint 重合（实测
   *  `odom → base_footprint` 为单位变换），平地地面 z ≈ 0，故取地面下方约
   *  0.2 m（≈ −0.2）。注意 `lidar_z` 是**雷达**在 odom 下的高度（平地为
   *  +0.230，即 base_footprint→front_mid360 的安装高度），不是本值的参考基准。
   *  下坡或地面下沉时，真实地面会低于该地板而被排除，坡面场景需重设。
   *  若换车或改安装，需按新实测值重设。 */
  double ground_floor_z = -0.2;  // 判据见 aboveGroundFloor（odom 绝对）

  /** @brief 高度网格（planar voxel）的边长；归格分辨率。 */
  double planar_voxel_size = 0.2;
};
