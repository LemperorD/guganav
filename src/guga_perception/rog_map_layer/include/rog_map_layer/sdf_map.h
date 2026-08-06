// =============================================================================
// sdf_map.h - 有符号距离场地图 (Signed Distance Field Map)
// =============================================================================
// SDFmap 类实现了一个基于2D占据栅格地图的环境表示方法，用于机器人导航：
// 1. 使用激光雷达点云通过射线投射(raycasting)更新占据栅格地图
// 2. 将占据信息转换为ESDF(Euclidean Signed Distance Field, 欧几里得有符号距离场)
// 3. 对外提供距离查询接口，支持任意位置到最近障碍物的距离查询
// 4. 支持双线性插值，获得连续平滑的距离值和梯度
// =============================================================================

#ifndef _SDF_MAP_H_
#define _SDF_MAP_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <queue>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <carstatemsgs/CarState.h>

#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>

#include <plan_env/raycast.h>

// logit 函数: 将概率 p 转换为对数几率 (log-odds) 形式
// 公式: logit(p) = ln(p / (1-p))
// 用于占据栅格地图的概率更新，将对数域运算转为加减法
#define logit(x) (log((x) / (1 - (x))))

class SDFmap;      // 有符号距离场地图
class PathSDFmap;  // 路径规划专用的SDF地图
class UnknownMap;   // 未知区域地图

// =============================================================================
// SDFmap 类: 有符号距离场环境地图
// =============================================================================
// 核心功能：
//   - 接收激光点云，通过占据栅格更新算法构建障碍物地图
//   - 基于占据栅格计算欧几里得有符号距离场 (ESDF)
//   - 提供距离查询、碰撞检测、梯度计算等接口
//   - 支持多种射线投射模式 (FOV限制、圆形补全等)
// =============================================================================
class SDFmap
{
  private:
    // =========================================================================
    // ROS 通信：定时器、发布者、订阅者
    // =========================================================================
    ros::NodeHandle nh_;                         // ROS节点句柄

    // --- 发布者 (Publishers) ---
    ros::Publisher pub_gridmap_;                 // 发布占据栅格地图，主题: /SDFmap/gridmap
    ros::Publisher pub_ESDF_;                    // 发布ESDF距离场，主题: /SDFmap/ESDF
    ros::Publisher pub_gradESDF_;                // 发布ESDF梯度场可视化，主题: /SDFmap/ESDFGrad

    // --- 定时器 (Timers) ---
    ros::Timer occ_timer_;                       // 占据栅格更新定时器 (50ms周期)
    ros::Timer esdf_timer_;                      // ESDF距离场更新定时器 (50ms周期)
    ros::Timer vis_timer_;                       // 可视化发布定时器 (500ms周期)

    // =========================================================================
    // 输入数据：激光雷达点云
    // =========================================================================
    ros::Subscriber cloud_sub_;                  // 订阅局部点云，主题: "local_pointcloud"
    bool has_cloud_ = false;                     // 是否已收到点云数据
    bool if_have_map_ = false;                   // 是否已有地图
    pcl::PointCloud<pcl::PointXYZ> cloud_;       // 存储最近一次收到的点云

    // =========================================================================
    // 里程计 (Odometry) / 机器人位姿
    // =========================================================================
    Eigen::Vector3d odom_pos_;                   // 机器人当前位置 (x, y, yaw)
    Eigen::Vector2d update_odom_;                // 用于触发地图更新的位置阈值
    bool has_odom_ = false;                      // 是否已收到里程计数据

    // =========================================================================
    // 状态标志
    // =========================================================================
    bool occ_need_update_ = false;               // 占据栅格是否需要更新
    bool has_map_ = false;                       // 占据栅格地图是否已初始化
    bool has_esdf_ = false;                      // ESDF距离场是否已计算
    bool esdf_need_update_ = false;              // ESDF距离场是否需要重新计算
    
    // =========================================================================
    // ESDF 距离缓存
    // =========================================================================
    // 存储全局地图中每个栅格的ESDF距离值(到最近障碍物的欧几里得距离)
    // 初始化为 double::max() 表示无限远
    std::vector<double> distance_buffer_all_;

    // =========================================================================
    // 位姿订阅 (替代里程计)
    // =========================================================================
    ros::Subscriber Pose_sub_;                   // 订阅机器人位姿信息

    // =========================================================================
    // 占据栅格地图核心数据结构
    // =========================================================================
    // gridmap_: 三态栅格地图 (Unknown=0, Unoccupied=1, Occupied=2)
    // 这是经过阈值化后的最终状态地图，供查询使用
    uint8_t *gridmap_ = nullptr;

    // =========================================================================
    // 占据栅格地图 - 对数几率 (Log-Odds) 概率模型
    // =========================================================================
    // 使用对数几率表示每个栅格的占据概率，避免数值不稳定。
    // 更新公式: L = L_prev + L_hit (命中) 或 L = L_prev + L_miss (未命中)
    // L 值大于 min_occupancy_log_ 时判定为占据 (Occupied)
    // 每个栅格维护两个计数器用于离群点去除 (outlier removal)
    //

    // 对数几率形式的占据概率值缓存，全局地图大小
    std::vector<double> occupancy_map_;

    // count_hit_: 每个栅格被"命中"(hit)的次数
    // count_hit_and_miss_: 每个栅格被射线穿过(hit+miss)的总次数
    // 用于离群点去除：当 hit/miss 比例过低时，认为该占据是噪音
    std::vector<short> count_hit_, count_hit_and_miss_;

    // 待更新的体素队列 (存储栅格索引)
    std::queue<Eigen::Vector2i> cache_voxel_;

    // 对数几率形式的概率参数 (从ROS参数服务器加载)
    double prob_hit_log_;                        // 命中(hit)的log-odds增量
    double prob_miss_log_;                       // 未命中(miss)的log-odds减量
    double clamp_max_log_;                       // log-odds上限 (防止数值溢出)
    double clamp_min_log_;                       // log-odds下限 (防止数值溢出)
    double unknown_flag_;                        // 未知区域的标记值 (0.01)
    double min_occupancy_log_;                   // 判定为占据的最小log-odds阈值

    // =========================================================================
    // 感知与射线投射参数
    // =========================================================================
    double detection_range_;                     // 点云检测范围/半径 (米)，默认5.0m

    bool if_perspective_;                        // 是否使用透视投影(视锥体)射线投射

    bool if_cirSupRaycast_;                      // 是否启用圆形补全射线投射
                                                 // 用于填补雷达数据中的空洞

    // =========================================================================
    // 调试可视化
    // =========================================================================
    ros::Publisher debug_maker;                  // 调试用MarkerArray发布者，主题: /SDFmap/debug

    // =========================================================================
    // 视场角 (FOV) 限制参数
    // =========================================================================
    bool hrz_limited_;                           // 是否限制水平视场角
    double hrz_laser_range_dgr_;                 // 水平激光扫描范围 (弧度)，默认 2*PI (360度)
                                                 // 从ROS参数加载时以角度为单位，构造时转为弧度

  public:

    // =========================================================================
    // 栅格状态枚举
    // =========================================================================
    // 三种栅格状态:
    //   Unknown    (0): 未知区域 - 未被传感器观测到
    //   Unoccupied (1): 空闲区域 - 射线穿过但未遇到障碍物
    //   Occupied   (2): 占据区域 - 激光点命中的障碍物位置
    enum {Unknown, Unoccupied, Occupied};

    // =========================================================================
    // 栅格地图几何参数 (local: 以机器人为中心的局部地图)
    // =========================================================================
    double grid_interval_;                       // 栅格边长 (米)，默认 0.1m (10cm)
    double inv_grid_interval_;                   // 栅格边长的倒数 (1/grid_interval_)，用于加速坐标转换

    double x_upper_ = -DBL_MAX, y_upper_ = -DBL_MAX;  // 局部地图上边界
	  double x_lower_ = DBL_MAX, y_lower_ = DBL_MAX;    // 局部地图下边界
    int X_SIZE_, Y_SIZE_, XY_SIZE_;                    // 局部地图尺寸: X方向栅格数, Y方向栅格数, 总栅格数

    
    // =========================================================================
    // 栅格地图几何参数 (global: 全局固定地图)
    // =========================================================================
    double global_x_upper_ = -DBL_MAX, global_y_upper_ = -DBL_MAX;  // 全局地图上边界 (米)
	  double global_x_lower_ = DBL_MAX, global_y_lower_ = DBL_MAX;    // 全局地图下边界 (米)
    int GLX_SIZE_, GLY_SIZE_;                         // 全局地图尺寸: X方向,Y方向栅格数
	  int GLXY_SIZE_;                                   // 全局地图总栅格数 (GLX_SIZE_ * GLY_SIZE_)
    Eigen::Vector2i EIXY_SIZE_;                      // 全局地图尺寸的Eigen向量形式

    // =========================================================================
    // 构造函数: 从ROS参数服务器加载配置，初始化所有发布者/订阅者/定时器
    // =========================================================================
    SDFmap(const ros::NodeHandle &nh){
      nh_ = nh;

      // --- 调试可视化发布者 ---
      // 发布 MarkerArray 用于在 RViz 中显示调试信息
      debug_maker = nh_.advertise<visualization_msgs::MarkerArray>("/SDFmap/debug",1);

      // --- 栅格分辨率参数 ---
      // gridmap_interval: 每个栅格的边长 (米)，默认 0.1m 即 10cm
      nh_.param<double>(ros::this_node::getName()+"/gridmap_interval",grid_interval_,0.1);
      inv_grid_interval_ = 1/grid_interval_;
  
      // --- 可视化发布者 ---
      // pub_gridmap_: 将占据栅格地图以点云形式发布到 /SDFmap/gridmap 主题，用于 RViz 可视化
      // pub_ESDF_:    将ESDF距离场以点云形式发布到 /SDFmap/ESDF 主题，用于 RViz 可视化
      // pub_gradESDF_: 将距离场梯度以箭头 MarkerArray 形式发布到 /SDFmap/ESDFGrad 主题
      pub_gridmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/SDFmap/gridmap",1);
      pub_ESDF_ = nh_.advertise<sensor_msgs::PointCloud2>("/SDFmap/ESDF",1);
      pub_gradESDF_ = nh_.advertise<visualization_msgs::MarkerArray>("/SDFmap/ESDFGrad",1);

      // --- 定时器 ---
      // occ_timer_:  50ms 周期调用 updateOccupancyCallback，更新占据栅格地图
      // esdf_timer_: 50ms 周期调用 updateESDFCallback，更新ESDF距离场
      // vis_timer_:  500ms 周期调用 visCallback，发布可视化数据
      occ_timer_ = nh_.createTimer(ros::Duration(0.05), &SDFmap::updateOccupancyCallback, this);
      esdf_timer_ = nh_.createTimer(ros::Duration(0.05), &SDFmap::updateESDFCallback, this);
      vis_timer_ = nh_.createTimer(ros::Duration(0.5), &SDFmap::visCallback, this);

      // --- 点云订阅 ---
      // 订阅局部点云主题 "local_pointcloud"，收到点云后触发 pointCloudCallback
      cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("local_pointcloud", 1, &SDFmap::pointCloudCallback, this);

      // --- 感知范围参数 ---
      // detection_range: 激光雷达/传感器的最大检测距离 (米)，默认 5.0m
      nh_.param<double>(ros::this_node::getName()+"/detection_range",detection_range_,5.0);
      
      // --- 全局地图边界参数 (米) ---
      // 定义全局固定地图的范围，默认 20m x 20m
      nh_.param<double>(ros::this_node::getName()+"/global_x_lower",global_x_lower_, -10);
      nh_.param<double>(ros::this_node::getName()+"/global_x_upper",global_x_upper_, 10);
      nh_.param<double>(ros::this_node::getName()+"/global_y_lower",global_y_lower_, -10);
      nh_.param<double>(ros::this_node::getName()+"/global_y_upper",global_y_upper_, 10);

      // --- 透视投影射线投射 ---
      // 是否使用视锥体 (frustum) 方式做射线投射，模拟相机 FOV
      nh_.param<bool>(ros::this_node::getName()+"/if_perspective",if_perspective_,false);

      // --- 圆形补全射线投射 ---
      // 如果启用，会在雷达数据中圆形采样以填补空洞
      nh_.param<bool>(ros::this_node::getName()+"/if_cirSupRaycast", if_cirSupRaycast_, false);

      // --- 水平视场角 (FOV) 限制 ---
      // hrz_limited:     是否限制水平扫描角度
      // hrz_laser_range_dgr: 水平扫描范围，默认 360 度，加载后转为弧度
      nh_.param<bool>(ros::this_node::getName()+"/hrz_limited",hrz_limited_,false);
      nh_.param<double>(ros::this_node::getName()+"/hrz_laser_range_dgr",hrz_laser_range_dgr_,360.0);
      hrz_laser_range_dgr_ = hrz_laser_range_dgr_ / 180.0 * M_PI ;
    
      
      // =========================================================================
      // 初始化地图内存
      // =========================================================================

      // --- 计算全局地图尺寸 ---
      // 根据边界和栅格分辨率计算 X, Y 方向的栅格数量
      GLX_SIZE_ = ceil((global_x_upper_ - global_x_lower_) / grid_interval_);
      GLY_SIZE_ = ceil((global_y_upper_ - global_y_lower_) / grid_interval_);
      GLXY_SIZE_ = GLX_SIZE_ * GLY_SIZE_;
      EIXY_SIZE_ << GLX_SIZE_, GLY_SIZE_;

      // --- 初始化三态栅格地图 (gridmap_) ---
      // 所有栅格初始化为 Unknown (0) 状态
      gridmap_ = new uint8_t[GLXY_SIZE_];
      memset(gridmap_, Unknown, GLXY_SIZE_ * sizeof(uint8_t));

      // --- 初始化 ESDF 距离缓存 ---
      // 所有距离初始化为 double::max() 表示无穷远
      distance_buffer_all_ = std::vector<double>(GLXY_SIZE_, std::numeric_limits<double>::max());

      // --- 计算局部地图尺寸 ---
      // 局部地图是以机器人为中心、边长 = 2*detection_range_ 的正方形区域
      X_SIZE_ = ceil(detection_range_ / grid_interval_) * 2;
      Y_SIZE_ = ceil(detection_range_ / grid_interval_) * 2;
      XY_SIZE_ = X_SIZE_ * Y_SIZE_;

      // =========================================================================
      // 占据栅格地图 - 对数几率模型参数初始化
      // =========================================================================
      // 从 ROS 参数服务器加载概率参数，并转换为对数几率 (log-odds) 形式
      // p_hit:  激光命中时的占据概率 (典型值 0.7)
      // p_miss: 激光穿过时的空闲概率 (典型值 0.4)
      // p_min:  概率下限 (典型值 0.12)
      // p_max:  概率上限 (典型值 0.97)
      // p_occ:  判定为占据的概率阈值 (典型值 0.7)
      double p_hit, p_miss, p_min, p_max, p_occ;
      nh_.getParam(ros::this_node::getName()+"/p_hit", p_hit);
      nh_.getParam(ros::this_node::getName()+"/p_miss", p_miss);
      nh_.getParam(ros::this_node::getName()+"/p_min", p_min);
      nh_.getParam(ros::this_node::getName()+"/p_max", p_max);
      nh_.getParam(ros::this_node::getName()+"/p_occ", p_occ);

      // 将概率转换为对数几率: logit(p) = ln(p/(1-p))
      prob_hit_log_ = logit(p_hit);
      prob_miss_log_ = logit(p_miss);
      clamp_min_log_ = logit(p_min);
      clamp_max_log_ = logit(p_max);
      min_occupancy_log_ = logit(p_occ);

      // unknown_flag_ = 0.01 用于标记未知区域，occupancy_map_ 初始化为略低于 clamp_min_log_
      unknown_flag_ = 0.01;
      occupancy_map_ = std::vector<double>(GLXY_SIZE_, clamp_min_log_ - unknown_flag_);

      // 初始化命中/未命中计数器 (用于离群点去除)
      count_hit_ = std::vector<short>(GLXY_SIZE_, 0);
      count_hit_and_miss_ = std::vector<short>(GLXY_SIZE_, 0);
    }

    // 析构函数: 释放 gridmap_ 动态数组内存
    ~SDFmap(){
      delete[] gridmap_;
      gridmap_ = nullptr;
    };

    bool get_grid_map_;

    // =========================================================================
    // ROS 回调函数
    // =========================================================================

    // 点云回调: 收到新的激光雷达点云时调用，存储点云并标记需要更新占据栅格
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    // 占据栅格更新定时器回调 (50ms): 对最新点云执行射线投射并更新占据概率
    void updateOccupancyCallback(const ros::TimerEvent& /*event*/);

    // 标准射线投射处理: 从机器人位置向每个激光点投射射线，更新沿线所有栅格的概率
    void raycastProcess();

    // 圆形补全射线投射处理: 在标准射线投射基础上，额外在圆形区域内采样以填补空洞
    void cirSupRaycastProcess();

    // 更新占据栅格地图: 根据累计的 log-odds 概率将栅格分类为 Unknown/Unoccupied/Occupied
    void updateOccupancyMap();

    // 离群点去除: 移除命中率过低的虚假障碍物 (通过 count_hit / count_hit_and_miss 比例判定)
    void RemoveOutliers();

    // ESDF 更新定时器回调 (50ms): 当占据栅格更新后，重新计算欧几里得有符号距离场
    void updateESDFCallback(const ros::TimerEvent& /*event*/);

    // 可视化定时器回调 (500ms): 定期发布占据栅格地图和 ESDF 的可视化数据
    void visCallback(const ros::TimerEvent& /*event*/);


    // =========================================================================
    // 坐标转换函数
    // =========================================================================
    // 栅格坐标系 (grid index) 与世界坐标系 (meter) 之间的相互转换
    // 转换公式: coord = lower_bound + (index + 0.5) * grid_interval_
    //             index = floor((coord - lower_bound) / grid_interval_)

    // 将2D栅格索引转换为世界坐标 (返回栅格中心点坐标)
    Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index);
    Eigen::Vector2d gridIndex2coordd(const int &x, const int &y);

    // 将世界坐标转换为2D栅格索引 (向下取整)
    Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt);

    // 将一维线性索引 (vector number) 转换为2D栅格索引
    Eigen::Vector2i vectornum2gridIndex(const int &num);

    // 将2D栅格索引转换为一维线性索引: num = y * GLX_SIZE_ + x
    int Index2Vectornum(const int &x, const int &y);
    int Index2Vectornum(const Eigen::Vector2i &index);

    // =========================================================================
    // 障碍物设置与碰撞检测
    // =========================================================================

    // 在指定3D/2D坐标处标记障碍物
    void setObs(const Eigen::Vector3d coord);
    void setObs(const Eigen::Vector2d coord);

    // 在全局地图中插入一个矩形障碍物 (box)
    // location: 障碍物中心位置 (世界坐标)
    // euler:    障碍物的欧拉角姿态 (R-P-Y)
    // size:     障碍物的长宽高尺寸
    inline void grid_insertbox(Eigen::Vector3d location, Eigen::Matrix3d euler, Eigen::Vector3d size);

    // 检查指定2D坐标处的栅格是否被占据 (返回栅格状态: Unknown/Unoccupied/Occupied)
    uint8_t CheckCollisionBycoord(const Eigen::Vector2d &pt);
    uint8_t CheckCollisionBycoord(const double ptx, const double pty);

    // 检查点是否在全局地图范围内
    bool isInGloMap(const Eigen::Vector2d &pt);

    // 找到地图中距离 pt 最近的有效点 (用于将出界点投影回地图边界内)
    Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos);

    // 设置缓存的占据状态 (将体素加入更新队列)
    int setCacheOccupancy(Eigen::Vector2d pos, int occ);
    int setCacheOccupancy(Eigen::Vector2i idx, int occ);

    // Bresenham 算法: 获取2D栅格直线上两点之间的所有栅格索引
    // 用于射线投射时确定射线穿过的所有栅格
    std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end);

    // =========================================================================
    // 栅格状态查询
    // =========================================================================

    bool isOccupied(const Eigen::Vector2i &index);           // 栅格是否被占据 (Occupied)
    bool isOccupied(const int &idx, const int &idy);
    bool isUnOccupied(const int &idx, const int &idy);       // 栅格是否空闲 (Unoccupied)
    bool isUnOccupied(const Eigen::Vector2i &index);
    bool isUnknown(const Eigen::Vector2i &index);            // 栅格是否未知 (Unknown)
    bool isUnknown(const int &idx, const int &idy);

    // 带安全距离的占据检查: 检查以 index 为中心、safe_dis 为半径的区域内是否有占据栅格
    // 用于路径规划中的碰撞检测，保证路径点与障碍物保持最小安全距离
    bool isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis);
    bool isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis);

    // =========================================================================
    // 可视化
    // =========================================================================
    void publish_gridmap();    // 将占据栅格地图以 PointCloud2 形式发布到 RViz

    // =========================================================================
    // ESDF (Euclidean Signed Distance Field) 欧几里得有符号距离场
    // =========================================================================
    // 计算每个栅格到最近障碍物的欧几里得距离。
    // 空闲区域 (Unoccupied) → 正距离 (到最近障碍物的最小距离)
    // 占据区域 (Occupied)   → 负距离 (取负值)
    // 未知区域 (Unknown)    → 距离未定义

    // 更新2D ESDF: 对全局栅格地图计算有符号距离场
    void updateESDF2d();

    // 通用的 ESDF 填充函数 (模板，支持1D/2D广播)
    // 沿指定维度 (dim=0 X方向, dim=1 Y方向) 传播距离值
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    void publish_ESDF();       // 发布 ESDF 距离场点云 (主题: /SDFmap/ESDF)
    void publish_ESDFGrad();   // 发布 ESDF 梯度场箭头 (主题: /SDFmap/ESDFGrad)

    // 查询指定栅格索引处的有符号距离值
    inline double getDistance(const Eigen::Vector2i& id);
    inline double getDistance(const int& idx, const int& idy);

    // =========================================================================
    // 双线性插值距离查询 (Bilinear Interpolation)
    // =========================================================================
    // 对于任意连续坐标 pos，用其所在栅格的4个邻居进行双线性插值，
    // 获得平滑连续的距离值和梯度，适用于优化器和路径规划中的连续空间查询

    // 查询距离并计算梯度 (距离, 梯度)
    double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad);

    // 查询距离并计算梯度，同时指定最小距离阈值 (mindis 用于截断)
    double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis);

    // 仅查询距离 (不计算梯度)
    double getDistWithGradBilinear(const Eigen::Vector2d &pos);

    // 查询未知区域的梯度 (用于引导机器人向已知区域探索)
    double getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad);

    // 获取指定世界坐标处到最近障碍物的实际欧几里得距离 (非插值)
    double getDistanceReal(const Eigen::Vector2d& pos);

    // 将世界坐标转换为 ESDF 查询用的栅格索引 (处理边界截断)
    inline Eigen::Vector2i ESDFcoord2gridIndex(const Eigen::Vector2d &pt);

    // 获取触发地图更新的里程计位置阈值
    Eigen::Vector2d get_update_odom();

    // 角度归一化: 将角度转换到 [-PI, PI] 范围
    inline double normalize_angle(double angle);

};


#endif
