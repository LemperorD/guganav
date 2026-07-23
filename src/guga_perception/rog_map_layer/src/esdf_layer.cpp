// Copyright 2026 guganav
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rog_map_layer/esdf_layer.hpp"

#include <nav2_costmap_2d/cost_values.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rog_map_layer
{

void EsdfLayer::onInitialize()
{
  auto node = node_.lock();

  // 基础参数
  enabled_ = true;
  current_ = false;

  // ESDF 距离参数
  config_.max_distance =
    node->declare_parameter(name_ + ".max_distance", config_.max_distance);
  config_.obstacle_threshold = static_cast<unsigned char>(
    node->declare_parameter(
      name_ + ".obstacle_threshold",
      static_cast<int>(config_.obstacle_threshold)));

  // 更新策略
  config_.incremental_update =
    node->declare_parameter(
    name_ + ".incremental_update",
    config_.incremental_update);
  config_.full_update_ratio =
    node->declare_parameter(
    name_ + ".full_update_ratio",
    config_.full_update_ratio);

  // TBB 并行参数
  config_.enable_parallel =
    node->declare_parameter(
    name_ + ".enable_parallel",
    config_.enable_parallel);
  if (config_.enable_parallel) {
    int default_threads = std::max(1, tbb::info::default_concurrency() - 1);
    config_.num_threads =
      node->declare_parameter(name_ + ".num_threads", default_threads);
    config_.tile_size =
      node->declare_parameter(name_ + ".tile_size", config_.tile_size);
    config_.seam_iterations =
      node->declare_parameter(
      name_ + ".seam_iterations",
      config_.seam_iterations);
    executor_ = std::make_unique<EsdfParallelExecutor>(config_.num_threads);
  }

  // 输出控制
  config_.publish_esdf_grid =
    node->declare_parameter(
    name_ + ".publish_esdf_grid",
    config_.publish_esdf_grid);
  config_.write_to_master =
    node->declare_parameter(
    name_ + ".write_to_master",
    config_.write_to_master);

  // ESDF 可视化发布
  if (config_.publish_esdf_grid) {
    esdf_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      name_ + "/esdf_grid", rclcpp::QoS(1).transient_local());
  }

  esdf_map_ = std::make_unique<EsdfMap>();

  needs_full_update_ = true;
  current_ = true;
}

void EsdfLayer::matchSize()
{
  auto * costmap = layered_costmap_->getCostmap();
  config_.resolution = costmap->getResolution();
  config_.size_x = costmap->getSizeInCellsX();
  config_.size_y = costmap->getSizeInCellsY();

  esdf_map_->resize(
    config_.size_x, config_.size_y, config_.resolution,
    costmap->getOriginX(), costmap->getOriginY());

  previous_occupancy_.resize(
    config_.size_x * config_.size_y,
    nav2_costmap_2d::NO_INFORMATION);
  needs_full_update_ = true;
}

void EsdfLayer::reset()
{
  esdf_map_->reset();
  std::fill(
    previous_occupancy_.begin(), previous_occupancy_.end(),
    nav2_costmap_2d::NO_INFORMATION);
  needs_full_update_ = true;
}

void EsdfLayer::updateBounds(
  double /*robot_x*/,
  double /*robot_y*/,
  double /*robot_yaw*/,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  auto * costmap = layered_costmap_->getCostmap();
  if (costmap->getSizeInMetersX() <= 0.0 || costmap->getSizeInMetersY() <= 0.0) {
    return;
  }

  // ESDF layer needs the full costmap every frame
  // distance field depends on global obstacle info
  *min_x = std::min(*min_x, costmap->getOriginX());
  *min_y = std::min(*min_y, costmap->getOriginY());
  *max_x = std::max(*max_x, costmap->getOriginX() + costmap->getSizeInMetersX());
  *max_y = std::max(*max_y, costmap->getOriginY() + costmap->getSizeInMetersY());
}

void EsdfLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_ || !esdf_map_) {
    return;
  }

  // Guard: matchSize() must have been called and the grid must be non-empty
  if (config_.size_x == 0 || config_.size_y == 0) {
    return;
  }

  const auto * master_data = master_grid.getCharMap();
  if (!master_data) {
    return;
  }

  size_t n = config_.size_x * config_.size_y;

  // Verify esdf_map_ internal buffers match the expected size
  if (esdf_map_->sizeX() != config_.size_x ||
    esdf_map_->sizeY() != config_.size_y)
  {
    return;
  }

  auto * executor_ptr = executor_.get();

  // 全量或增量 ESDF 计算
  if (needs_full_update_ || !config_.incremental_update) {
    esdf_map_->computeFull(master_data, config_, executor_ptr);
    needs_full_update_ = false;
  } else {
    esdf_map_->computeIncremental(
      master_data, previous_occupancy_,
      config_, executor_ptr);
  }

  // 保存当前帧的占用快照（供后续增量检测）
  std::memcpy(previous_occupancy_.data(), master_data, n);

  // 将 ESDF 代价写入 master grid
  if (config_.write_to_master) {
    writeCostsToMaster(master_grid, min_i, min_j, max_i, max_j);
  }

  // 发布 ESDF 可视化
  if (config_.publish_esdf_grid && esdf_pub_) {
    auto grid = esdf_map_->toOccupancyGrid(
      layered_costmap_->getGlobalFrameID(),
      clock_->now(),
      config_);
    esdf_pub_->publish(grid);
  }

  current_ = true;
}

const EsdfMap * EsdfLayer::getEsdfMapRaw() const
{
  return esdf_map_.get();
}

float EsdfLayer::getDistance(double wx, double wy) const
{
  if (!esdf_map_) {
    return 0.0f;
  }
  return esdf_map_->getDistanceAt(wx, wy);
}

std::pair<float, float> EsdfLayer::getGradient(double wx, double wy) const
{
  if (!esdf_map_) {
    return {0.0f, 0.0f};
  }
  return esdf_map_->getGradientAt(wx, wy);
}

unsigned char EsdfLayer::distanceToCost(
  float distance,
  const EsdfConfig & config)
{
  if (distance <= 0.0f) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  if (distance >= static_cast<float>(config.max_distance)) {
    return nav2_costmap_2d::FREE_SPACE;
  }
  float ratio = distance / static_cast<float>(config.max_distance);
  return static_cast<unsigned char>(
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE * (1.0f - ratio));
}

void EsdfLayer::writeCostsToMaster(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  size_t size_x = esdf_map_->sizeX();
  size_t map_h = esdf_map_->sizeY();

  if (size_x == 0 || map_h == 0) {
    return;
  }

  // Clamp to valid bounds
  int clamped_min_i = std::max(0, min_i);
  int clamped_min_j = std::max(0, min_j);
  int clamped_max_i = std::min(static_cast<int>(size_x), max_i);
  int clamped_max_j = std::min(static_cast<int>(map_h), max_j);

  // Guard against negative or zero-sized update window
  if (clamped_max_i <= clamped_min_i || clamped_max_j <= clamped_min_j) {
    return;
  }

  // Use serial path for now — TBB parallel writes to master_grid
  // can cause segfaults since setCost() is not thread-safe
  for (int j = clamped_min_j; j < clamped_max_j; ++j) {
    for (int i = clamped_min_i; i < clamped_max_i; ++i) {
      size_t idx = static_cast<size_t>(j) * size_x + static_cast<size_t>(i);
      float dist = esdf_map_->getDistance(idx);
      master_grid.setCost(
        static_cast<unsigned int>(i),
        static_cast<unsigned int>(j),
        distanceToCost(dist, config_));
    }
  }
}

}  // namespace rog_map_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rog_map_layer::EsdfLayer, nav2_costmap_2d::Layer)
