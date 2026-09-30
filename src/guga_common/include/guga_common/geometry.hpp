#ifndef GUGA_COMMON_GEOMETRY_HPP
#define GUGA_COMMON_GEOMETRY_HPP

namespace guga_common {

/**
 * @brief 三维点：三个 double 的 xyz，单位与坐标系由使用方约定。
 *
 * 只表示一个位置，不含姿态；需要姿态时由使用方另行携带（角度或四元数）。
 * 保持聚合类型，可用 `{x, y, z}` 初始化。
 */
struct Point3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

}  // namespace guga_common

#endif  // GUGA_COMMON_GEOMETRY_HPP
