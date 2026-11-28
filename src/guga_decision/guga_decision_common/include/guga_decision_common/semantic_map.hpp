#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "guga_decision_common/waypoint_navigator.hpp"

namespace guga_decision_common
{

// 一条隧道。字段由 launch 校验后通过参数传入，本类不再做合法性检查。
struct SemanticRegion
{
  std::pair<double, double> p0{0.0, 0.0};
  std::pair<double, double> p1{0.0, 0.0};
  double half_width{0.0};      // 横向半宽，米
  double approach_margin{0.0};  // 进入前多少米开始准备，米
};

// 前瞻结果：命中的区域，以及从路径起点算起的弧长。
struct Trigger
{
  const SemanticRegion* region{nullptr};
  double distance{0.0};
};

// 几何辅助。与 ROS 无关，便于单独做单元测试。
namespace geometry
{

// 点到线段的距离，米。两端点重合时退化为点到点。
double pointToSegmentDistance(double px, double py,
                              const std::pair<double, double>& a,
                              const std::pair<double, double>& b);

// 到区域边界的距离，米。点在隧道内时为 0。
double regionDistance(const SemanticRegion& region, double x, double y);

}  // namespace geometry

class SemanticMap : public rclcpp::Node
{
public:
  explicit SemanticMap(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // 参数是否齐全。合法性由 launch 保证，这里只挡“点数不对”。
  [[nodiscard]] bool valid() const { return valid_; }

  // 返回半径内命中的区域。区域距离定义为“到中线的距离减半宽”，最小为 0。
  std::vector<const SemanticRegion*> near(double x, double y,
                                          double radius) const;

  // 沿路径前瞻，返回首次进入隧道的点与弧长。已在隧道内返回距离 0；
  // 前瞻范围内没遇到返回空。
  std::optional<Trigger> next_morph_trigger(
      const std::vector<Waypoint2D>& path, double lookahead) const;

private:
  // 发布一条 LINE_STRIP，用于在 RViz 里核对位置。
  void publishMarkers();

  SemanticRegion region_;
  bool valid_{false};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace guga_decision_common
