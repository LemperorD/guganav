#include "guga_decision_common/semantic_map.hpp"

#include <algorithm>
#include <cmath>

namespace guga_decision_common {
  namespace {
    // 沿路径采样步长，米。越小触发点越准，代价是每次查询多跑几轮。
    constexpr double SAMPLE_STEP{0.05};
  }  // namespace

  namespace geometry {

    double pointToSegmentDistance(double px, double py,
                                  const std::pair<double, double>& a,
                                  const std::pair<double, double>& b) {
      const double abx = b.first - a.first;
      const double aby = b.second - a.second;
      const double len2 = abx * abx + aby * aby;
      if (len2 <= 1e-12) {
        return std::hypot(px - a.first, py - a.second);
      }
      double t = ((px - a.first) * abx + (py - a.second) * aby) / len2;
      t = std::clamp(t, 0.0, 1.0);
      return std::hypot(px - (a.first + t * abx), py - (a.second + t * aby));
    }

    double regionDistance(const SemanticRegion& region, double x, double y) {
      const double center = pointToSegmentDistance(x, y, region.p0, region.p1);
      return std::max(0.0, center - region.half_width);
    }

  }  // namespace geometry

  std::vector<const SemanticRegion*> SemanticMap::near(double x, double y,
                                                       double radius) const {
    std::vector<const SemanticRegion*> hits;
    if (valid_ && geometry::regionDistance(region_, x, y) <= radius) {
      hits.push_back(&region_);
    }
    return hits;
  }

  std::optional<Trigger> SemanticMap::next_morph_trigger(
      const std::vector<Waypoint2D>& path, double lookahead) const {
    if (!valid_ || path.size() < 2 || lookahead <= 0.0)
      return std::nullopt;

    double travelled = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const auto& from = path[i - 1];
      const auto& to = path[i];
      const double segment = std::hypot(to.x - from.x, to.y - from.y);
      const int steps = std::max(
          1, static_cast<int>(std::ceil(segment / SAMPLE_STEP)));
      for (int k = 0; k <= steps; ++k) {
        const double ratio = static_cast<double>(k) / steps;
        const double arc = travelled + segment * ratio;
        if (arc > lookahead)
          return std::nullopt;
        const double x = from.x + (to.x - from.x) * ratio;
        const double y = from.y + (to.y - from.y) * ratio;
        if (geometry::regionDistance(region_, x, y) <= 0.0) {
          return Trigger{&region_, arc};
        }
      }
      travelled += segment;
    }
    return std::nullopt;
  }

}  // namespace guga_decision_common
