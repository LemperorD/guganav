/**
 * @file common_lib.h
 * @brief Point-LIO 公共类型定义和工具函数
 * @author HKU-MARS (original), LihanChen2004 (fork)
 *
 * 本文件定义了 Point-LIO 系统中所有模块共享的：
 * - 流形类型别名 (基于 IKFoM/MTK 模板库)
 * - 状态流形定义 (input/output 两种模式)
 * - 过程噪声流形定义
 * - 常用宏、Eigen 类型别名
 * - MeasureGroup 数据结构 (雷达-IMU帧同步)
 * - 平面估计等模板工具函数
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <point_lio/so3_math.h>
#include <tf2_ros/transform_broadcaster.h>

#include <IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <sensor_msgs/msg/imu.hpp>

using namespace std;
using namespace Eigen;

// ==================== MTK 流形类型别名 ====================
typedef MTK::vect<3, double> vect3;           ///< 3维向量流形
typedef MTK::SO3<double> SO3;                 ///< SO(3) 旋转群流形
typedef MTK::S2<double, 98090, 10000, 1> S2;  ///< 2维球面流形 (重力方向)
typedef MTK::vect<1, double> vect1;           ///< 1维向量流形
typedef MTK::vect<2, double> vect2;           ///< 2维向量流形

/**
 * @defgroup manifold_defs 状态流形定义 (MTK_BUILD_MANIFOLD)
 *
 * IMU 数据作为系统输入 (控制量), 类似于 FAST-LIO 经典模式
 * 角速度和加速度被估计而非输入
 *
 * 两种模式共用相同的 input_ikfom 输入流形 (3维加速度 + 3维角速度)
 * @{
 */

/** @brief 24维状态流形 — IMU-as-input 模式
 *
 * 状态分量 (索引):
 *   0-2:   pos          — 位置 (世界坐标系)
 *   3-5:   rot          — 姿态 SO(3)
 *   6-8:   offset_R_L_I — 雷达→IMU 外参旋转
 *   9-11:  offset_T_L_I — 雷达→IMU 外参平移
 *   12-14: vel          — 速度 (世界坐标系)
 *   15-17: bg           — 陀螺仪零偏
 *   18-20: ba           — 加速度计零偏
 *   21-23: gravity      — 重力向量 (世界坐标系)
 */
MTK_BUILD_MANIFOLD(state_input, ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                                    (vect3, offset_T_L_I))((vect3, vel))((
                                    vect3, bg))((vect3, ba))((vect3, gravity)));

/** @brief 30维状态流形 — IMU-as-output 模式
 *
 * 相比 input 模式额外包含:
 *   12-14: omg          — 角速度 (被估计)
 *   15-17: acc          — 加速度 (被估计)
 *   重力索引变为 21-23, 零偏索引变为 24-29
 */
MTK_BUILD_MANIFOLD(state_output,
                   ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                       (vect3, offset_T_L_I))((vect3, vel))((vect3, omg))(
                       (vect3, acc))((vect3, gravity))((vect3, bg))((vect3,
                                                                     ba)));

/** @brief 6维输入流形 — IMU 测量值 (两者共用) */
MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));

/** @brief 12维过程噪声流形 — IMU-as-input 模式
 *  ng: 陀螺仪白噪声, na: 加速度计白噪声,
 *  nbg: 陀螺仪零偏随机游走, nba: 加速度计零偏随机游走
 */
MTK_BUILD_MANIFOLD(process_noise_input,
                   ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

/** @brief 15维过程噪声流形 — IMU-as-output 模式
 *  额外包含 vel 噪声 (角速度+加速度本身被估计而非作为输入)
 */
MTK_BUILD_MANIFOLD(
    process_noise_output,
    ((vect3, vel))((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

/** @} */  // manifold_defs

// ==================== 数值常量 ====================
#define PBWIDTH 30  ///< 进度条宽度
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"

#define PI_M (3.14159265358)
// #define G_m_s2 (9.81)         // Gravaty const in GuangDong/China
#define DIM_STATE (24)        ///< 状态维度 (SO(3) 按 3 维计算)
#define DIM_PROC_N (12)       ///< 过程噪声维度
#define CUBE_LEN (6.0)        ///< 局部地图立方体边长
#define LIDAR_SP_LEN (2)      ///< 激光雷达特殊长度阈值
#define INIT_COV (0.0001)     ///< 初始协方差默认值
#define NUM_MATCH_POINTS (5)  ///< 点到面匹配所需的最小近邻点数
#define MAX_MEAS_DIM (10000)  ///< 单次量测最大维度

// ==================== 常用宏 ====================
#define CONSTRAIN(v, min, max) \
  ((v > min) ? ((v < max) ? v : max) : min)  ///< 值限幅
#define ARRAY_FROM_EIGEN(mat) \
  mat.data(), mat.data() + mat.rows() * mat.cols()  ///< Eigen→原始数组
#define STD_VEC_FROM_EIGEN(mat)  \
  vector<decltype(mat)::Scalar>( \
      mat.data(),                \
      mat.data() + mat.rows() * mat.cols())  ///< Eigen→std::vector

// ==================== PCL/Eigen 类型别名 ====================
using PointType =
    pcl::PointXYZINormal;  ///< PCL curvature 字段存储点时间偏移 (ms)

// PCL 固定字段名为 curvature；Point-LIO 通过语义化访问器将其作为时间偏移使用。
inline float point_time_offset_ms(const PointType& point) {
  return point.curvature;
}

inline void set_point_time_offset_ms(PointType& point, float offset_ms) {
  point.curvature = offset_ms;
}

using PointTypeRGB = pcl::PointXYZRGB;              ///< 带颜色的点类型
using PointCloudXYZI = pcl::PointCloud<PointType>;  ///< 常用点云类型
using PointCloudXYZRGB = pcl::PointCloud<PointTypeRGB>;
using PointVector =
    std::vector<PointType,
                Eigen::aligned_allocator<PointType>>;  ///< 对齐点向量
using V3D = Eigen::Vector3d;                           ///< 双精度3维向量
using M3D = Eigen::Matrix3d;                           ///< 双精度3x3矩阵
using V3F = Eigen::Vector3f;                           ///< 单精度3维向量
using M3F = Eigen::Matrix3f;                           ///< 单精度3x3矩阵

// ==================== std::vector → Eigen 转换 ====================
/** @brief std::vector<double> → Eigen::Vector3d (带长度校验) */
inline V3D to_vec3d(const std::vector<double>& v) {
  if (v.size() < 3) {
    throw std::runtime_error("to_vec3d: 输入长度不足 3");
  }
  return Eigen::Map<const V3D>(v.data());
}

/** @brief std::vector<double> → Eigen::Matrix3d (带长度校验) */
inline M3D to_mat3d(const std::vector<double>& v) {
  if (v.size() < 9) {
    throw std::runtime_error("to_mat3d: 输入长度不足 9");
  }
  return Eigen::Map<const M3D>(v.data());
}

#define MD(a, b) Matrix<double, (a), (b)>  ///< 动态大小双精度矩阵
#define VD(a) Matrix<double, (a), 1>       ///< 动态大小双精度列向量
#define MF(a, b) Matrix<float, (a), (b)>   ///< 动态大小单精度矩阵
#define VF(a) Matrix<float, (a), 1>        ///< 动态大小单精度列向量

/** @brief 预定义的常用常量矩阵 */
const M3D Eye3d(M3D::Identity());  ///< 3x3 单位阵 (double)
const M3F Eye3f(M3F::Identity());  ///< 3x3 单位阵 (float)
const V3D Zero3d(0, 0, 0);         ///< 3维零向量 (double)
const V3F Zero3f(0, 0, 0);         ///< 3维零向量 (float)

/**
 * @brief 当前处理帧的雷达+IMU数据组合
 *
 * 由 sync_packages() 函数填充:
 * - lidar: 降采样后的当前帧点云
 * - imu: 两帧之间的 IMU 测量队列
 * - lidar_beg_time: 当前帧起始时间 (秒)
 * - lidar_last_time: 当前帧结束时间 (秒)
 */
struct MeasureGroup {
  double lidar_start_time{0.0};  ///< 当前帧起始时间戳
  double lidar_last_time{0.0};   ///< 当前帧最后一个点的时间戳
  PointCloudXYZI::Ptr lidar;     ///< 降采样后的点云
  deque<sensor_msgs::msg::Imu::ConstSharedPtr>
      imu;  ///< 该帧时间范围内的IMU数据
};

// ==================== 工具函数 ====================

/**
 * @brief 计算两点之间的欧氏距离平方
 * @tparam T 返回类型 (double 或 float)
 * @param p1 第一个点
 * @param p2 第二个点
 * @return 距离平方值 ||p1 - p2||²
 */
template <typename T>
T calc_dist(PointType p1, PointType p2) {
  T d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
        + (p1.z - p2.z) * (p1.z - p2.z);
  return d;
}

/**
 * @brief 计算 Vector3d 到 PointType 的欧氏距离平方
 */
template <typename T>
T calc_dist(Eigen::Vector3d p1, PointType p2) {
  T d = (p1(0) - p2.x) * (p1(0) - p2.x) + (p1(1) - p2.y) * (p1(1) - p2.y)
        + (p1(2) - p2.z) * (p1(2) - p2.z);
  return d;
}

/**
 * @brief 时间压缩: 按时间戳分组点云 (用于逐组点 Kalman 更新)
 *
 * 点云的 PCL curvature 存储字段用于保存该点相对帧首的时间偏移 (ms)。
 * 此函数根据时间偏移的单调性将点云分成多个组，
 * 返回每个组的大小序列。
 *
 * 分组逻辑:
 * - 遍历点云，当点时间偏移递增时 (同组内时间递增)，累加计数
 * - 当点时间偏移回跳时 (新的一组开始)，记录当前组大小并重置计数
 *
 * @tparam T 计数值类型
 * @param point_cloud 输入点云 (point time offset = 时间偏移，单位 ms)
 * @return 每组包含的点数序列
 */
template <typename T>
std::vector<int> time_compressing(const PointCloudXYZI::Ptr& point_cloud) {
  int points_size = point_cloud->points.size();
  int j = 0;
  std::vector<int> time_seq;
  time_seq.reserve(points_size);
  for (int i = 0; i < points_size - 1; i++) {
    j++;
    // 当点时间偏移回跳时，开始新的分组
    if (point_time_offset_ms(point_cloud->points[i + 1])
        > point_time_offset_ms(point_cloud->points[i])) {
      time_seq.emplace_back(j);
      j = 0;
    }
  }
  // 最后一组
  {
    time_seq.emplace_back(j + 1);
  }
  return time_seq;
}

/**
 * @brief 通过最小二乘法估计平面法向量
 *
 * 平面方程: Ax + By + Cz + D = 0
 * 归一化: A/D*x + B/D*y + C/D*z = -1
 * 构造最小二乘问题: A0 * x0 = b0
 *   其中 A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1,...,-1]^T
 * 求解后归一化得到单位法向量
 *
 * @tparam T 浮点类型
 * @param[out] normvec 输出归一化法向量
 * @param point 输入点集
 * @param threshold 平面内点距离阈值
 * @param point_num 参与估计的点数
 * @return true 如果所有点在阈值内 (平面有效), false 否则
 */
template <typename T>
bool esti_normvector(Matrix<T, 3, 1>& normvec, const PointVector& point,
                     const T& threshold, const int& point_num) {
  MatrixXf A(point_num, 3);
  MatrixXf b(point_num, 1);
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < point_num; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }
  // 最小二乘求解 A/D, B/D, C/D
  normvec = A.colPivHouseholderQr().solve(b);

  // 验证所有点是否满足平面方程
  for (int j = 0; j < point_num; j++) {
    if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y
             + normvec(2) * point[j].z + 1.0f)
        > threshold) {
      return false;
    }
  }

  normvec.normalize();
  return true;
}

/**
 * @brief 用 5 个最近邻点估计局部平面 (固定点数版本)
 *
 * 平面方程: n·p + d = 0, 其中 n 为单位法向量, d 为截距
 * 返回 pca_result = [nx, ny, nz, d]^T
 *
 * @tparam T 浮点类型
 * @param[out] pca_result 4维平面参数 (nx, ny, nz, d)
 * @param point 5个最近邻点
 * @param threshold 平面一致性阈值
 * @return true 如果5个点都在阈值内(平面有效), false 否则
 */
template <typename T>
bool esti_plane(Matrix<T, 4, 1>& pca_result, const PointVector& point,
                const T& threshold) {
  Matrix<T, NUM_MATCH_POINTS, 3> A;
  Matrix<T, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

  // 归一化法向量并计算截距 d = 1/|x0|
  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;

  // 一致性检查: 所有5个点到平面的距离必须在阈值内
  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y
             + pca_result(2) * point[j].z + pca_result(3))
        > threshold) {
      return false;
    }
  }
  return true;
}

/**
 * @brief ROS2 Time 消息 → 秒 (double)
 */
inline double get_time_sec(const builtin_interfaces::msg::Time& time) {
  return rclcpp::Time(time).seconds();
}

/**
 * @brief 秒 (double) → ROS2 Time 消息
 */
inline rclcpp::Time get_ros_time(double timestamp) {
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}