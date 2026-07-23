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

#pragma once

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>
#include <vector>

#include "rog_map_layer/esdf_config.hpp"
#include "rog_map_layer/esdf_map.hpp"
#include "rog_map_layer/esdf_parallel.hpp"

namespace rog_map_layer
{

class EsdfLayer : public nav2_costmap_2d::Layer
{
public:
  EsdfLayer() = default;
  ~EsdfLayer() override = default;

  // Nav2 Layer 接口
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override;
  void reset() override;
  bool isClearable() override {return false;}

  // ESDF query API (for external planners/controllers)
  [[nodiscard]] const EsdfMap * getEsdfMapRaw() const;
  [[nodiscard]] float getDistance(double wx, double wy) const;
  [[nodiscard]] std::pair<float, float> getGradient(double wx, double wy) const;

  [[nodiscard]] const EsdfConfig & config() const {return config_;}

private:
  static unsigned char distanceToCost(float distance, const EsdfConfig & config);

  void writeCostsToMaster(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  EsdfConfig config_;
  std::unique_ptr<EsdfMap> esdf_map_;
  std::unique_ptr<EsdfParallelExecutor> executor_;

  std::vector<unsigned char> previous_occupancy_;
  bool needs_full_update_{true};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_pub_;
};

}  // namespace rog_map_layer
