#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>
#include <utility>
#include <vector>

#include "rog_map_layer/esdf_config.hpp"
#include "rog_map_layer/esdf_parallel.hpp"

namespace rog_map_layer
{

class EsdfMap
{
public:
  EsdfMap() = default;

  // 初始化ESDF地图
  void resize(size_t size_x, size_t size_y, double resolution, double origin_x, double origin_y);

  // 重置ESDF地图
  void reset();

  // 全量更新
  void computeFull(
    const unsigned char * occupancy,
    const EsdfConfig & config,
    EsdfParallelExecutor * executor = nullptr);

  // 增量更新
  void computeIncremental(
    const unsigned char * occupancy,
    const std::vector<unsigned char> & previous_occupancy,
    const EsdfConfig & config,
    EsdfParallelExecutor * executor = nullptr);

  inline int coordToIndex(size_t x, size_t y) const {return static_cast<int>(y * size_x_ + x);}

  // 获取 ESDF 代价和梯度
  [[nodiscard]] float getDistance(size_t idx) const;
  [[nodiscard]] std::pair<float, float> getGradient(size_t idx) const;
  [[nodiscard]] float getDistanceAt(double wx, double wy) const;
  [[nodiscard]] std::pair<float, float> getGradientAt(double wx, double wy) const;

  [[nodiscard]] bool worldToMap(double wx, double wy, size_t & mx, size_t & my) const;

  [[nodiscard]] const std::vector<float> & distanceField() const {return distance_field_;}
  [[nodiscard]] const std::vector<float> & gradientX() const {return gradient_x_;}
  [[nodiscard]] const std::vector<float> & gradientY() const {return gradient_y_;}
  [[nodiscard]] size_t sizeX() const {return size_x_;}
  [[nodiscard]] size_t sizeY() const {return size_y_;}
  [[nodiscard]] double resolution() const {return resolution_;}
  [[nodiscard]] double originX() const {return origin_x_;}
  [[nodiscard]] double originY() const {return origin_y_;}

  nav_msgs::msg::OccupancyGrid toOccupancyGrid(
    const std::string & frame_id, rclcpp::Time stamp,
    const EsdfConfig & config) const;

  bool isOccupied(size_t mx, size_t my) const;
  bool isUnoccupied(size_t mx, size_t my) const;
  bool isOccWithSafeDis(size_t mx, size_t my, double safe_dis) const;

private:
  static constexpr double INF_DISTANCE = std::numeric_limits<double>::infinity();

  void initFromOccupancy(
    const unsigned char * occupancy,
    unsigned char obstacle_threshold,
    EsdfParallelExecutor * executor = nullptr);

  void propagateForward(
    size_t start_row, size_t end_row,
    size_t start_col, size_t end_col);

  void propagateBackward(
    size_t start_row, size_t end_row,
    size_t start_col, size_t end_col);

  void copyOffsetsToDistances(const EsdfConfig & config);

  void computeGradients(EsdfParallelExecutor * executor = nullptr);

  // 分块并行 8SED
  void computeFullParallel(
    const unsigned char * occupancy,
    const EsdfConfig & config,
    EsdfParallelExecutor * executor);

  void computeFullSerial(
    const unsigned char * occupancy,
    const EsdfConfig & config);

  struct Tile
  {
    size_t row_start{}, row_end{};
    size_t col_start{}, col_end{};
  };

  std::vector<Tile> buildTiles(int tile_size) const;

  void fixSeamBetween(const Tile & a, const Tile & b, bool horizontal);

  void detectChanges(
    const unsigned char * occupancy,
    const std::vector<unsigned char> & previous,
    std::vector<size_t> & changed_indices) const;

  void expandDirtyRegion(
    const std::vector<size_t> & changed_indices,
    std::vector<bool> & dirty_mask,
    const EsdfConfig & config) const;

  // 8SED 使用 int16_t 偏移量存储最近障碍物的 (dx, dy)
  // distance = sqrt(dx^2 + dy^2) * resolution_
  std::vector<int16_t> offset_dx_; // 最近障碍物的 x 偏移量 (单位: cells)
  std::vector<int16_t> offset_dy_; // 最近障碍物的 y 偏移量 (单位: cells)

  // ESDF 距离场和梯度场
  std::vector<float> distance_field_;
  std::vector<float> gradient_x_;
  std::vector<float> gradient_y_;

  size_t size_x_{};
  size_t size_y_{};
  double resolution_{};
  double origin_x_{};
  double origin_y_{};
};

}  // namespace rog_map_layer
