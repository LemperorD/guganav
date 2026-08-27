/**
 * @file preprocess.h
 * @brief 多型号 LiDAR 点云预处理模块
 *
 * Preprocess 类支持以下雷达:
 * - **AVIA**: 览沃 Avia (Livox 非重复扫描, CustomMsg 格式)
 * - **VELO16**: Velodyne VLP-16 (机械旋转式, pointcloud2 格式)
 * - **OUST64**: Ouster OS1-64 (数字雷达, pointcloud2 格式)
 * - **HESAIxt32**: 禾赛 XT32 (机械旋转式, pointcloud2 格式)
 *
 * 预处理流程:
 * 1. 输入点云解析 (不同雷达格式 → 统一 PointType)
 * 2. 盲区/远距滤波 (blind < dist < det_range)
 * 3. 降采样 (每隔 point_filter_num 取1点)
 * 4. 特征提取 — 区分平面点和边缘点 (机械式雷达)
 * 5. 时间戳提取/计算 (PCL curvature 字段存储 ms 单位偏移时间)
 * 6. 切帧/合帧支持 (cut_frame / con_frame)
 *
 * 编码规范遵循 **模式 A** (函数式数据流),
 * 参数由 parameters.cpp 的 readParameters() 统一设置。
 */

#pragma once
#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_lio/common_lib.h"

using namespace std;

// ==================== 雷达类型枚举 ====================

/** @brief LiDAR 硬件类型标识 */
enum LidType : uint8_t { AVIA = 1, VELO16, OUST64, HESA_IXT32 };

/** @brief 时间戳单位 (用于解析不同雷达的时间格式) */
enum TimeUnit : uint8_t { SEC = 0, MS = 1, US = 2, NS = 3 };

struct PreprocessParams {
  int lidar_type{AVIA};
  int point_filter_num{1};
  int scan_lines{6};
  int scan_rate{10};
  int timestamp_unit{MS};
  double blind{1.0};
  double det_range{300.0};
};

// ==================== 时间排序谓词 ====================

/**
 * @brief 按点时间偏移升序排序 (切帧使用)
 */
bool time_list_cut_frame(const PointType& x, const PointType& y);

// ==================== 各雷达原生点结构 (PCL注册) ====================
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;  ///< x, y, z
    float intensity;  ///< 反射强度
    float time;       ///< 时间戳 (相对扫描周期)
    uint16_t ring;    ///< 激光线号 (0-15)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,  // NOLINT
                                            intensity)(

        float, time, time)(std::uint16_t, ring, ring))

namespace hesai_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;  ///< 绝对时间戳 (GPS时间)
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace hesai_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
    hesai_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(std::uint16_t, ring, ring))

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;             ///< 纳秒时间戳
    uint16_t reflectivity;  ///< 反射率
    uint8_t ring;           ///< 激光线号
    uint16_t ambient;       ///< 环境光
    uint32_t range;         ///< 原始距离测量 (mm)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x) //NOLINT
    (float, y, y) //NOLINT
    (float, z, z) //NOLINT
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on

// ==================== Preprocess 类 ====================

/**
 * @class Preprocess
 * @brief 多型号 LiDAR 点云预处理类
 *
 * 提供统一的预处理接口:
 * - process(msg, pcl_out): 标准处理 (特征提取/滤波/降采样)
 * - process_cut_frame_livox/process_cut_frame_pcl2: 切帧处理
 *
 * 内部根据 lidar_type 分发到不同的私有 handler:
 * - avia_handler: 处理 Livox 非重复扫描
 * - velodyne_handler: 处理 Velodyne 机械扫描
 * - oust64_handler: 处理 Ouster 数字雷达
 * - hesai_handler: 处理禾赛机械扫描
 */
class Preprocess {
public:
  /**
   * @brief Livox 切帧处理: 将一帧非重复扫描切分为多个子帧
   * @param msg Livox 点云消息
   * @param[out] pcl_out 输出的子帧点云队列
   * @param[out] time_lidar 各子帧的时间戳 (ms)
   * @param required_frame_num 需要切分的子帧数
   * @param scan_count 已接收帧数 (前5帧不切分)
   */
  void processCutFrameLivox(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
      deque<PointCloudXYZI::Ptr>& pcl_out, deque<double>& time_lidar,
      int required_frame_num, int scan_count);

  /**
   * @brief 标准雷达切帧处理 (Velodyne/Ouster/Hesai)
   * @see process_cut_frame_livox
   */
  constexpr static int MAX_LINE_NUM = 128;
  void processCutFramePCL2(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                           deque<PointCloudXYZI::Ptr>& pcl_out,
                           deque<double>& time_lidar, int required_frame_num,
                           int scan_count);

  /** @brief Livox 雷达标准处理 (输入 Livox CustomMsg) */
  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
               PointCloudXYZI::Ptr& pcl_out);

  /** @brief 标准雷达处理 (输入 ROS2 PointCloud2) */
  void process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
               PointCloudXYZI::Ptr& pcl_out);

  void configure(const PreprocessParams& params);

private:
  bool given_offset_time_{false};  ///< 是否有硬件提供的时间戳
  int lidar_type_{AVIA};           ///< 雷达类型 (LID_TYPE 枚举)
  int point_filter_num_{1};        ///< 降采样间隔 (每 N 点取 1)
  int n_scans_{6};           ///< 扫描线数, Livox 默认 6 线 (实际由 YAML 覆盖)
  int scan_rate_{10};        ///< 扫描频率 (Hz)
  int time_unit_{MS};        ///< 时间戳单位 (TIME_UNIT 枚举)
  float time_unit_scale_{};  ///< 时间单位缩放因子
  double det_range_{1000};  ///< 最大检测距离 (米)
  double blind_{0.01};      ///< 盲区距离 (米, 此距离内的点被过滤)

  PointCloudXYZI pl_full_;  ///< 完整点云 (未滤波)
  PointCloudXYZI pl_surf_;  ///< 平面点 (最终输出)

  /** @brief Livox Avia/Horizon/Mid-360 点云处理 (非重复扫描格式) */
  void aviaHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

  /** @brief Ouster OS1-64 点云处理 */
  void oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

  /** @brief Velodyne VLP-16 点云处理 (含时间戳估算) */
  void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

  /** @brief 禾赛 XT32 点云处理 */
  void hesaiHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

};
