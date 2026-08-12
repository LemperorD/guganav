// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include "terrain_analysis_ext/core/algorithm.hpp"
#include "terrain_analysis_ext/core/state.hpp"

#include <cmath>

namespace terrain_analysis_ext {
  namespace algorithm {

    void runExt(double local_terrain_map_radius, TerrainExtState& state) {
      state.has_new_laser_cloud = false;
      state.terrain_cloud_elev->clear();
      mergeLocalTerrain(local_terrain_map_radius, state);
    }

    void mergeLocalTerrain(double local_terrain_map_radius,
                           TerrainExtState& state) {
      for (const auto& point : state.terrain_cloud_local->points) {
        double distance = horizontalDistanceTo(point.x, point.y, state);
        if (distance <= local_terrain_map_radius) {
          state.terrain_cloud_elev->push_back(point);
        }
      }
    }

    [[nodiscard]] double horizontalDistanceTo(double px, double py,
                                              const TerrainExtState& state) {
      return std::sqrt(((px - state.vehicle_x) * (px - state.vehicle_x))
                       + ((py - state.vehicle_y) * (py - state.vehicle_y)));
    }

  }  // namespace algorithm
}  // namespace terrain_analysis_ext
