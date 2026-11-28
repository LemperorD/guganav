#include "guga_decision_common/semantic_map.hpp"

#include <cmath>
#include <vector>

namespace guga_decision_common
{
namespace
{
// 贴地薄片的厚度，米。只为了让它在 RViz 里有个面，不表示隧道高度。
constexpr double MARKER_THICKNESS{0.01};
}  // namespace

SemanticMap::SemanticMap(const rclcpp::NodeOptions& options)
    : rclcpp::Node("semantic_map", options)
{
  const auto p0 =
      declare_parameter<std::vector<double>>("p0", std::vector<double>{});
  const auto p1 =
      declare_parameter<std::vector<double>>("p1", std::vector<double>{});
  region_.half_width = declare_parameter<double>("half_width", 0.0);
  region_.approach_margin = declare_parameter<double>("approach_margin", 0.0);

  if (p0.size() == 2 && p1.size() == 2) {
    region_.p0 = {p0[0], p0[1]};
    region_.p1 = {p1[0], p1[1]};
    valid_ = true;
    RCLCPP_INFO(get_logger(), "隧道 (%.2f,%.2f)->(%.2f,%.2f)，半宽 %.2f，提前量 %.2f",
                region_.p0.first, region_.p0.second, region_.p1.first,
                region_.p1.second, region_.half_width, region_.approach_margin);
  } else {
    // 正常由 launch 启动，参数在 launch 里读 yaml 并校验过。
    RCLCPP_ERROR(get_logger(),
                 "p0/p1 必须各给两个数；请使用 launch/semantic_map.launch.py");
  }

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/semantic_map", rclcpp::QoS(1).transient_local());

  if (valid_) publishMarkers();
}

void SemanticMap::publishMarkers()
{
  // 贴地的薄片：绕 z 轴转到隧道走向，z 方向只留 1 厘米厚度，
  // 所以上下表面始终与 map 的 z 轴垂直，从任何角度看都不倾斜。
  const double dx = region_.p1.first - region_.p0.first;
  const double dy = region_.p1.second - region_.p0.second;
  const double length = std::hypot(dx, dy);
  const double psi = std::atan2(dy, dx);

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker box;
  box.header.frame_id = "map";
  box.header.stamp = now();
  box.ns = "semantic_map";
  box.id = 0;
  box.type = visualization_msgs::msg::Marker::CUBE;
  box.action = visualization_msgs::msg::Marker::ADD;
  box.pose.position.x = (region_.p0.first + region_.p1.first) / 2.0;
  box.pose.position.y = (region_.p0.second + region_.p1.second) / 2.0;
  box.pose.position.z = MARKER_THICKNESS / 2.0;  // 底面贴地
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = std::sin(psi / 2.0);
  box.pose.orientation.w = std::cos(psi / 2.0);
  box.scale.x = length;
  box.scale.y = 2.0 * region_.half_width;
  box.scale.z = MARKER_THICKNESS;
  box.color.r = 1.0F;
  box.color.g = 1.0F;
  box.color.b = 0.0F;
  box.color.a = 0.5F;
  array.markers.push_back(box);
  marker_pub_->publish(array);
}

}  // namespace guga_decision_common
