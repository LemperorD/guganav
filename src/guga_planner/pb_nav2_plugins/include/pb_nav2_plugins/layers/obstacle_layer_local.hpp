#pragma once

#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace pb_nav2_costmap_2d {

/**
 * @brief 本仓库自己的障碍层，插件名带 `local` 后缀，与官方
 * `nav2_costmap_2d::ObstacleLayer` 区分开。
 *
 * 现在只做继承：行为与官方层一致，但插件的名字与归属在本仓，便于按需覆写官方层
 * 的虚接口（`onInitialize` / `updateBounds` / `updateCosts` / `activate` /
 * `deactivate` / `reset` / `raytraceFreespace` 都是虚的），而不必把官方那 700 多行
 * 源码整份拷进来维护。
 *
 * 需要"按 intensity（距局部地面高度）过滤点"的话，本包已有
 * `pb_nav2_costmap_2d::IntensityVoxelLayer`——它就是为这件事从官方层派生出来的。
 */
class ObstacleLayerLocal : public nav2_costmap_2d::ObstacleLayer {
public:
  ObstacleLayerLocal() = default;
  ~ObstacleLayerLocal() override = default;
};

}  // namespace pb_nav2_costmap_2d
