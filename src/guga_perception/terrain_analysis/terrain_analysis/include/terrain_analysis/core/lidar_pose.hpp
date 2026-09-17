#pragma once

/**
 * @brief 雷达在 odom 坐标系下的位姿中与几何有关的部分。
 *
 * 单独成一个类型而不是三个标量：这三者总是同时出现（网格索引基准、横向距离、
 * 相对高度、滚动窗口都以它为参照），拆开传递容易出现"传了 x 忘了 z"这类错误，
 * 也会让同一概念在状态与各组件之间出现多种形态。
 *
 * 注意：odom 原点与 base_footprint 重合，平地时 `z` 即雷达安装高度
 * （≈ +0.230 m），地面在 z ≈ 0。
 *
 * 与 TerrainConfig / TerrainState 一样位于全局命名空间——本包的数据结构沿用
 * 这一约定，算法与阶段在 terrain_analysis 命名空间内。
 */
struct LidarPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};
