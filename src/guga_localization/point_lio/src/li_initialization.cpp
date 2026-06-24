/**
 * @file li_initialization.cpp
 * @brief LiDAR-IMU 初始化和传感器数据同步实现
 *
 * 核心功能:
 * 1. **雷达回调**: 接收 Livox/标准点云，调用 preprocess 进行特征提取/滤波，
 *    支持 con_frame (合帧) 和 cut_frame_init (切帧) 模式
 * 2. **IMU 回调**: 接收 IMU 数据，进行时间戳校准后加入队列
 * 3. **sync_packages()**: 将雷达帧与时间匹配的 IMU 数据打包成 MeasureGroup，
 *    这是主循环每次处理的基本数据单元
 *
 * 数据流向:
 *   LiDAR 驱动 ──→ standard_pcl_cbk / livox_pcl_cbk ──→ lidar_buffer
 *   IMU 驱动  ──→ imu_cbk                             ──→ imu_deque
 *                                 sync_packages ──→ MeasureGroup ──→ 主循环处理
 */

#include "li_initialization.h"

// ==================== 初始化状态 ====================
bool data_accum_finished = false;       ///< 数据累积完成
bool data_accum_start = false;          ///< 数据累积开始
bool online_calib_finish = false;       ///< 在线标定完成
bool refine_print = false;              ///< 精标定打印
int frame_num_init = 0;                 ///< 初始化帧数
double time_lag_IMU_wtr_lidar = 0.0;    ///< IMU 相对 LiDAR 时间延迟
double move_start_time = 0.0;
double online_calib_starts_time = 0.0;
double imu_first_time = 0.0;
bool lose_lid = false;                  ///< 激光帧丢失标志
double timediff_imu_wrt_lidar = 0.0;    ///< IMU→LiDAR 固有时差
bool timediff_set_flg = false;          ///< 时差是否已配置
V3D gravity_lio = V3D::Zero();          ///< LIO 重力估计

// ==================== 线程同步 ====================
mutex mtx_buffer;                       ///< 缓冲互斥锁
sensor_msgs::msg::Imu imu_last;          ///< 上帧 IMU
sensor_msgs::msg::Imu imu_next;          ///< 下帧 IMU
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());  ///< 合帧累积点云

// ==================== 调试数组 ====================
double T1[MAXN];                        ///< 时间戳数组
double s_plot[MAXN];                    ///< 总耗时
double s_plot2[MAXN];                   ///< 特征点数
double s_plot3[MAXN];                   ///< 平均耗时
double s_plot11[MAXN];                  ///< 预处理耗时

condition_variable sig_buffer;          ///< 缓冲区条件变量
int scan_count = 0;                     ///< 接收帧数
int frame_ct = 0;                       ///< 合帧计数
int wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false;              ///< 雷达帧已推入
bool imu_pushed = false;                ///< IMU 已推入
std::deque<PointCloudXYZI::Ptr> lidar_buffer;       ///< 雷达帧缓冲队列
std::deque<double> time_buffer;                     ///< 雷达时间戳缓冲队列
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;  ///< IMU 数据缓冲队列

/**
 * @brief 标准 ROS2 点云回调 — Velodyne/Ouster/Hesai 等雷达
 *
 * 处理步骤:
 * 1. 时间戳回环检测: 如果当前帧时间 < 上一帧时间，丢弃 (回放时可能发生)
 * 2. 切帧模式 (cut_frame_init): 调用 process_cut_frame_pcl2() 将一帧切为多个子帧
 * 3. 常规模式: 调用 process() 提取点云特征
 * 4. 合帧模式 (con_frame): 累积 con_frame_num 帧合并为一帧发布
 * 5. 入队: 将处理后的帧加入 lidar_buffer / time_buffer
 */
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  scan_count++;
  double preprocess_start_time = omp_get_wtime();

  // ---- 时间戳回环检测 ----
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  // ---- 切帧模式 (初始化阶段切分帧以加速收敛) ----
  if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));  // ms → s
      timestamp_lidar.pop_front();
    }
  } else {
    // ---- 常规处理 ----
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));
    p_pre->process(msg, ptr);

    // ---- 合帧模式: 累积多帧 ----
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;  // 记录合帧起始时间
      }
      if (frame_ct < 10) {
        // 将 curvature 偏移以保留时间信息: 秒→毫秒，加上相对于合帧头的时间差
        for (int i = 0; i < ptr->size(); i++) {
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        // 累积完成，作为一帧发布
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        *ptr_con_i = *ptr_con;
        lidar_buffer.push_back(ptr_con_i);
        double time_con_i = time_con;
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      // ---- 单帧模式: 直接入队 ----
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }

  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
}

/**
 * @brief Livox 自定义点云回调 — Avia/Mid-360/Horizon 系列
 *
 * 处理逻辑与 standard_pcl_cbk 相同，主要区别:
 * - 输入格式: livox_ros_driver2::msg::CustomMsg
 * - 切帧: 调用 process_cut_frame_livox()
 * - 常规处理: 使用 avia_handler 提取特征 (直线列扫描，非机械旋转)
 * - 时间单位: Livox offset_time 为纳秒，需转换为毫秒存于 curvature
 */
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
{
  double preprocess_start_time = omp_get_wtime();
  scan_count++;

  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  // ---- 切帧模式 ----
  if (cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));  // ms → s
      timestamp_lidar.pop_front();
    }
  } else {
    // ---- 常规 Livox 处理 (avia_handler) ----
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
    p_pre->process(msg, ptr);

    // ---- 合帧模式: 累积多帧 (Livox 帧率较高，10帧合1帧) ----
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;
      }
      if (frame_ct < 10) {
        for (int i = 0; i < ptr->size(); i++) {
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        *ptr_con_i = *ptr_con;
        double time_con_i = time_con;
        lidar_buffer.push_back(ptr_con_i);
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      // ---- 单帧直接入队 ----
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }

  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
}

/**
 * @brief IMU 数据回调
 *
 * 执行时间戳校准 (两条链):
 *   t_calibrated = t_orig - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar
 *
 * - timediff_imu_wrt_lidar: 固有时钟差 (通过 NTP/PTP 同步后的残余)
 * - time_lag_IMU_wtr_lidar: 动态估计的传输延迟
 *
 * 回环检测: 时间戳倒流则清空队列 (避免 rossbag 回放时的异常)
 */
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in)
{
  // 复制消息 (便于修改时间戳)
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  // ---- 时间戳校准: 减去时间偏差 ----
  msg->header.stamp = get_ros_time(
    get_time_sec(msg_in->header.stamp) - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

  double timestamp = get_time_sec(msg->header.stamp);

  // ---- 时间戳回环检测 ----
  if (timestamp < last_timestamp_imu) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "imu loop back, clear deque");
    return;
  }
  imu_deque.emplace_back(msg);
  last_timestamp_imu = timestamp;
}

/**
 * @brief LiDAR-IMU 数据同步核心函数
 *
 * 每次主循环调用此函数，尝试从缓冲队列中组装一组匹配的 LiDAR-IMU 数据:
 *
 * **IMU 禁用模式** (imu_en = false):
 *   - 直接从 lidar_buffer 取一帧
 *   - 遍历点云找最大 curvature (最远点偏移时间), 计算 lidar_end_time
 *   - 打包为 MeasureGroup, 不包含 IMU 数据
 *
 * **IMU 启用模式** (imu_en = true):
 *   1. 等待两个队列都有数据
 *   2. lidar_pushed=false时: 取一帧雷达，计算 lidar_end_time
 *   3. 等待: last_timestamp_imu >= lidar_end_time (IMU 覆盖整个帧时间)
 *   4. imu_pushed=false时: 将 [lidar_beg_time, lidar_end_time] 内的 IMU 数据打包到 meas.imu
 *   5. 弹出已消费的雷达帧，返回 true
 *
 * **lose_lid 机制**: 当雷达帧点数 < 1 时，标记 lose_lid=true，
 * 使用 lidar_time_inte 作为预期帧长度来收集足够的 IMU 数据。
 *
 * @param[out] meas 组装好的 MeasureGroup (雷达帧 + IMU 队列)
 * @return true 成功组装一组数据, false 数据不足继续等待
 */
bool sync_packages(MeasureGroup & meas)
{
  {
    // ==================== IMU 禁用模式 ====================
    if (!imu_en) {
      if (!lidar_buffer.empty()) {
        if (!lidar_pushed) {
          meas.lidar = lidar_buffer.front();
          meas.lidar_beg_time = time_buffer.front();
          lose_lid = false;

          if (meas.lidar->points.empty()) {
            std::cout << "lose lidar" << '\n';
            lose_lid = true;
          } else {
            // 遍历点云找到最大的 curvature (最远点偏移时间)
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt : meas.lidar->points) {
              if (pt.curvature > end_time) {
                end_time = pt.curvature;
              }
            }
            // 帧结束时间 = 帧头时间 + 最远点偏移 (ms转s)
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
          }
          lidar_pushed = true;
        }

        // 弹出已处理的帧
        time_buffer.pop_front();
        lidar_buffer.pop_front();
        lidar_pushed = false;

        return !lose_lid;
      }
      return false;
    }

    // ==================== IMU 启用模式 ====================
    // 1. 检查数据是否就绪
    if (lidar_buffer.empty() || imu_deque.empty()) {
      return false;
    }

    // 2. 取雷达帧 (如果还没取)
    if (!lidar_pushed) {
      lose_lid = false;
      meas.lidar = lidar_buffer.front();
      meas.lidar_beg_time = time_buffer.front();

      if (meas.lidar->points.size() < 1) {
        std::cout << "lose lidar" << '\n';
        lose_lid = true;  // 空帧标记为丢失
      } else {
        // 计算帧结束时间 (最远点时间)
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt : meas.lidar->points) {
          if (pt.curvature > end_time) {
            end_time = pt.curvature;
          }
        }
        // lidar_end_time = 帧起始时间 + 扫描周期 (ms转s)
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
        meas.lidar_last_time = lidar_end_time;
      }
      lidar_pushed = true;
    }

    // 3. 等待 IMU 数据覆盖完整帧时间区间
    if (!lose_lid && (last_timestamp_imu < lidar_end_time)) {
      return false;  // IMU 数据还不够多
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) {
      return false;  // 丢帧时: 用 lidar_time_inte 估算帧长度
    }

    // 4. 打包 IMU 数据 (帧头→帧尾 时间范围内的 IMU)
    if (!lose_lid && !imu_pushed) {
      if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        imu_next = *(imu_deque.front());
        meas.imu.shrink_to_fit();

        // 收集帧时间范围内的所有 IMU 数据
        while (imu_time < lidar_end_time) {
          meas.imu.emplace_back(imu_deque.front());
          imu_last = imu_next;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_time = get_time_sec(imu_deque.front()->header.stamp);
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;
    }

    // 4b. 丢帧时的 IMU 打包 (使用 lidar_time_inte 估算)
    if (lose_lid && !imu_pushed) {
      if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();

        imu_next = *(imu_deque.front());
        while (imu_time < meas.lidar_beg_time + lidar_time_inte) {
          meas.imu.emplace_back(imu_deque.front());
          imu_last = imu_next;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_time = get_time_sec(imu_deque.front()->header.stamp);
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;
    }

    // 5. 清理: 弹出已处理的帧，重置标志
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
  }
}