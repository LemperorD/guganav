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
 * 5. 时间戳提取/计算 (curvature 域存储 ms 单位偏移时间)
 * 6. 切帧/合帧支持 (cut_frame / con_frame)
 *
 * 编码规范遵循 **模式 A** (函数式数据流),
 * 参数由 parameters.cpp 的 readParameters() 统一设置。
 */

#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

/// @brief 有效性检查 (未使用)
#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// ==================== 雷达类型枚举 ====================

/** @brief LiDAR 硬件类型标识 */
enum LID_TYPE { AVIA = 1, VELO16, OUST64, HESAIxt32 };

/** @brief 时间戳单位 (用于解析不同雷达的时间格式) */
enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };

// ==================== 特征类型枚举 (用于机械旋转式雷达) ====================

/** @brief 点特征分类 */
enum Feature {
  Nor,         ///< 未分类
  Poss_Plane,  ///< 候选平面点 (端点)
  Real_Plane,  ///< 确认平面点 (内部)
  Edge_Jump,   ///< 跳变边缘
  Edge_Plane,  ///< 平面交界边缘
  Wire,        ///< 细小物体 (电线等)
  ZeroPoint    ///< 零点
};

/** @brief 临域方向 (Prev=前向, Next=后向) */
enum Surround { Prev, Next };

/** @brief 跳变边缘分类 */
enum E_jump {
  Nr_nor,   ///< 正常
  Nr_zero,  ///< 零距离
  Nr_180,   ///< 180° 夹角 (遮挡边界)
  Nr_inf,   ///< 无穷远
  Nr_blind  ///< 盲区
};

// ==================== 时间排序谓词 ====================

/**
 * @brief 按 curvature (时间偏移) 升序排序 (切帧使用)
 */
const bool time_list_cut_frame(PointType & x, PointType & y);

// ==================== 点特征数据结构 ====================

/**
 * @struct orgtype
 * @brief 存储每个点的几何特征属性
 *
 * 用于机械旋转式雷达 (Velodyne/Ouster/Hesai) 的特征提取:
 * - range: 到原点的距离
 * - dista: 到相邻点连线的垂直距离 (平面性指标)
 * - angle[2]: 与前后点的夹角 (cos值)
 * - edj[2]: 前后方向的跳变边缘类型
 * - ftype: 该点最终特征类型
 */
struct orgtype
{
  double range;      ///< 到雷达原点的距离
  double dista;      ///< 到前后点连线的垂直距离
  double angle[2];   ///< 与前后邻点的夹角 cos值
  double intersect;  ///< 前后连线的夹角 cos值
  E_jump edj[2];     ///< 前后跳变类型
  Feature ftype;     ///< 最终特征类型

  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;  // 初始小角度 (>1 即 cos初始值无效)
  }
};

// ==================== 各雷达原生点结构 (PCL注册) ====================

namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;       ///< x, y, z
  float intensity;       ///< 反射强度
  float time;            ///< 时间戳 (相对扫描周期)
  uint16_t ring;         ///< 激光线号 (0-15)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  velodyne_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                         float, time, time)(std::uint16_t, ring, ring))

namespace hesai_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;      ///< 绝对时间戳 (GPS时间)
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace hesai_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  hesai_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                      double, timestamp, timestamp)(std::uint16_t, ring, ring))

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;            ///< 纳秒时间戳
  uint16_t reflectivity; ///< 反射率
  uint8_t ring;          ///< 激光线号
  uint16_t ambient;      ///< 环境光
  uint32_t range;        ///< 原始距离测量 (mm)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
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
class Preprocess
{
public:
  Preprocess();
  ~Preprocess();

  /**
   * @brief Livox 切帧处理: 将一帧非重复扫描切分为多个子帧
   * @param msg Livox 点云消息
   * @param[out] pcl_out 输出的子帧点云队列
   * @param[out] time_lidar 各子帧的时间戳 (ms)
   * @param required_frame_num 需要切分的子帧数
   * @param scan_count 已接收帧数 (前5帧不切分)
   */
  void process_cut_frame_livox(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg,
    deque<PointCloudXYZI::Ptr> & pcl_out, deque<double> & time_lidar,
    const int required_frame_num, int scan_count);

  /**
   * @brief 标准雷达切帧处理 (Velodyne/Ouster/Hesai)
   * @see process_cut_frame_livox
   */
  void process_cut_frame_pcl2(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    deque<PointCloudXYZI::Ptr> & pcl_out, deque<double> & time_lidar,
    const int required_frame_num, int scan_count);

  /** @brief Livox 雷达标准处理 (输入 Livox CustomMsg) */
  void process(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg, PointCloudXYZI::Ptr & pcl_out);

  /** @brief 标准雷达处理 (输入 ROS2 PointCloud2) */
  void process(const sensor_msgs::msg::PointCloud2::SharedPtr & msg, PointCloudXYZI::Ptr & pcl_out);

  /** @brief 设置预处理参数 */
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // ==================== 公有成员变量 ====================

  PointCloudXYZI pl_full;     ///< 完整点云 (未滤波)
  PointCloudXYZI pl_corn;     ///< 角点/边缘点
  PointCloudXYZI pl_surf;     ///< 平面点 (最终输出)
  PointCloudXYZI pl_buff[128]; ///< 按线号缓存的原始点 (最多128线)
  vector<orgtype> typess[128]; ///< 按线号存储的特征类型

  float time_unit_scale;      ///< 时间单位缩放因子
  int lidar_type;             ///< 雷达类型 (LID_TYPE 枚举)
  int point_filter_num;       ///< 降采样间隔 (每 N 点取 1)
  int N_SCANS;                ///< 扫描线数
  int SCAN_RATE;              ///< 扫描频率 (Hz)
  int time_unit;              ///< 时间戳单位 (TIME_UNIT 枚举)
  double blind;               ///< 盲区距离 (米, 此距离内的点被过滤)
  double det_range;           ///< 最大检测距离 (米)
  bool given_offset_time;     ///< 是否有硬件提供的时间戳

private:
  /** @brief Livox Avia/Horizon/Mid-360 点云处理 (非重复扫描格式) */
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg);

  /** @brief Ouster OS1-64 点云处理 */
  void oust64_handler(const sensor_msgs::msg::PointCloud2::SharedPtr & msg);

  /** @brief Velodyne VLP-16 点云处理 (含时间戳估算) */
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr & msg);

  /** @brief 禾赛 XT32 点云处理 */
  void hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr & msg);

  /**
   * @brief 特征分类主函数: 区分平面点/边缘点/细小物体
   *
   * 使用三维几何特征: 相邻点的距离变化、法向量夹角、跳变检测等
   * 将点云中的点分为 Real_Plane / Poss_Plane / Edge_Jump / Edge_Plane / Wire
   *
   * @param pl 输入点云
   * @param types 输入/输出特征类型数组
   */
  void give_feature(PointCloudXYZI & pl, vector<orgtype> & types);

  /**
   * @brief 判断一组连续点是否构成平面
   *
   * 使用法向量稳定性指标:
   *   - 点组内每个点到首末点连线的垂直距离 (dista)
   *   - 距离方差比值 (用于排除边缘点)
   *   - 激光束方向的平面投影宽度/长度比
   *
   * @param pl 输入点云
   * @param types 特征类型数组
   * @param i_cur 当前起始点索引
   * @param[out] i_nex 输出平面结束点索引
   * @param[out] curr_direct 输出平面方向向量 (首→末)
   * @return 1=平面, 0=非平面, 2=盲区
   */
  int plane_judge(
    const PointCloudXYZI & pl, vector<orgtype> & types, uint i_cur, uint & i_nex,
    Eigen::Vector3d & curr_direct);

  /**
   * @brief 小型平面的二次判定 (精化平面边界)
   *
   * 补充 plane_judge 的遗漏: 跳变边缘附近的短平面段
   */
  bool small_plane(
    const PointCloudXYZI & pl, vector<orgtype> & types, uint i_cur, uint & i_nex,
    Eigen::Vector3d & curr_direct);

  /**
   * @brief 跳变边缘真伪判定
   *
   * 防止将遮挡导致的"假边缘"误分为特征点
   * 条件: 前后点到激光的距离突变 + 角度连续性
   *
   * @param nor_dir 判断方向 (Prev/Next)
   */
  bool edge_jump_judge(const PointCloudXYZI & pl, vector<orgtype> & types, uint i, Surround nor_dir);

  // ==================== 特征提取参数 (经验值) ====================
  int group_size;            ///< 点组大小 (8)
  double disA, disB;         ///< 距离容限: disA*range + disB
  double inf_bound;          ///< 无穷远边界 (>此值视为盲区?)
  double limit_maxmid;       ///< AVIA: dis_max/dis_mid 上限
  double limit_midmin;       ///< AVIA: dis_mid/dis_min 上限
  double limit_maxmin;       ///< 机械雷达: dis_max/dis_min 上限
  double p2l_ratio;          ///< 投影宽度/长度比值上限
  double jump_up_limit;      ///< 跳变角度上限 (cos 170°)
  double jump_down_limit;    ///< 跳变角度下限 (cos 8°)
  double cos160;             ///< 160° cos(160°)
  double edgea, edgeb;       ///< 跳变边缘判定参数
  double smallp_intersect;   ///< 小平面的前后夹角阈值 cos(172.5°)
  double smallp_ratio;       ///< 小平面距离比阈值
  double vx, vy, vz;         ///< 当前方向向量缓存 (plane_judge 中使用)
};