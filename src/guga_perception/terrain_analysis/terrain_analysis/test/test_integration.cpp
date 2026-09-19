// 集成测试：只通过 ROS 话题驱动节点，不直接调用 ingest / update / compute /
// 体素地图。
//
// 覆盖的是单元测试看不到的那一层：话题订阅与消息转换（Odometry → 雷达位置、
// PointCloud2 → PCL 点云与时间戳）、节点的逐帧数据分发（前半段收帧与累积、
// 采集结果交给后半段）、以及发布出去的消息本身（frame_id、stamp、intensity
// 语义）。
//
// 场景与 measure_leaf 的测量场景一致（±5 m 地面 0.1 m 网格、0.2 m 宽的竖向
// 方块），因此 measure_leaf 测得的行为（幽灵点 4 帧消失）在本测试里应重现。

#include "terrain_analysis/terrain_analysis_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <thread>

namespace terrain_analysis {

  namespace {

    /** @brief 帧间隔（10 Hz），与 measure_leaf 相同。 */
    constexpr double kFrameDt = 0.1;
    /** @brief 地面网格的半边长与间距。 */
    constexpr double kGroundHalf = 5.0;
    constexpr double kGroundSpacing = 0.1;
    /** @brief 方块几何：6 列 × 16 行，顶面 0.35 m，宽 0.2 m。 */
    constexpr int kBoxCols = 6;
    constexpr int kBoxRows = 16;

    /**
     * @brief 造一帧合成扫描：以车为中心的地面 + 可选方块。
     *
     * 地面覆盖车辆 ±kGroundHalf，因此车辆移动后地面随之移动；方块固定在世界
     * 坐标 box_x 处，box_x < 0 表示本帧没有方块（幽灵点场景用）。
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr makeFrame(double vehicle_x,
                                                   double vehicle_y,
                                                   double box_x) {
      auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      const int cells = static_cast<int>(2.0 * kGroundHalf / kGroundSpacing);
      for (int ix = 0; ix <= cells; ix++) {
        for (int iy = 0; iy <= cells; iy++) {
          pcl::PointXYZI p;
          p.x = static_cast<float>(vehicle_x - kGroundHalf
                                   + ix * kGroundSpacing);
          p.y = static_cast<float>(vehicle_y - kGroundHalf
                                   + iy * kGroundSpacing);
          p.z = 0.0F;
          p.intensity = 0.0F;
          cloud->push_back(p);
        }
      }
      if (box_x >= 0.0) {
        for (int ix = 0; ix < kBoxCols; ix++) {
          for (int iz = 0; iz < kBoxRows; iz++) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(box_x - 0.1 + ix * 0.04);
            p.y = 0.0F;
            p.z = static_cast<float>(0.05 + iz * 0.02);
            p.intensity = 0.0F;
            cloud->push_back(p);
          }
        }
      }
      return cloud;
    }

    /** @brief 统计点云里落在给定平面邻域内的点数。 */
    std::size_t countNear(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                          double x, double y, double radius) {
      std::size_t n = 0;
      for (const auto& p : cloud.points) {
        if (std::hypot(p.x - x, p.y - y) <= radius) {
          n++;
        }
      }
      return n;
    }

    /** @brief 邻域内点的最大 intensity（即最大离地高度）。 */
    float maxIntensityNear(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                           double x, double y, double radius) {
      float value = 0.0F;
      for (const auto& p : cloud.points) {
        if (std::hypot(p.x - x, p.y - y) <= radius) {
          value = std::max(value, p.intensity);
        }
      }
      return value;
    }

  }  // namespace

  /**
   * @brief 通过话题驱动 TerrainAnalysis 节点的集成测试夹具。
   *
   * 节点与测试驱动节点挂在同一个单线程执行器上：测试发布 odom 与点云，
   * 节点的 10 ms 定时器驱动 processOnce，发布出来的 terrain_map 由测试订阅
   * 收下。每一帧都等到"本帧已被处理并发布"再进入下一帧，因此不存在把两帧
   * 合成一帧的竞态。
   */
  class TerrainMapIntegrationTest : public testing::Test {
  protected:
    void SetUp() override {
      rclcpp::init(0, nullptr);
      // 执行器必须在 init 之后构造：它的 guard condition 需要有效的上下文。
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

      rclcpp::NodeOptions options;
      options.parameter_overrides(
          {rclcpp::Parameter("decayTime", 0.5),
           rclcpp::Parameter("noDecayDis", 0.0),
           rclcpp::Parameter("useSorting", true),
           rclcpp::Parameter("quantileZ", 0.2),
           rclcpp::Parameter("minObstacleHeight", 0.04),
           rclcpp::Parameter("ceilingClearance", 0.62)});
      terrain_ = std::make_shared<TerrainAnalysis>(options);

      driver_ = std::make_shared<rclcpp::Node>("terrain_map_test_driver");
      pub_odom_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
          "/lidar_odometry", 5);
      pub_scan_ = driver_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/registered_scan", 5);
      sub_map_ = driver_->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/terrain_map", 10,
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            last_map_ = msg;
            map_count_++;
          });

      executor_->add_node(terrain_);
      executor_->add_node(driver_);

      // 等话题匹配完成，避免第一帧发布时订阅还没建立。
      const bool matched = spinUntil(
          [this]() {
            return pub_scan_->get_subscription_count() > 0
                   && sub_map_->get_publisher_count() > 0;
          },
          std::chrono::milliseconds(5000));
      ASSERT_TRUE(matched) << "话题未在超时前匹配，测试环境有问题";
    }

    void TearDown() override {
      sub_map_.reset();
      pub_odom_.reset();
      pub_scan_.reset();
      executor_->remove_node(driver_);
      executor_->remove_node(terrain_);
      executor_.reset();
      driver_.reset();
      terrain_.reset();
      rclcpp::shutdown();
    }

    /** @brief 反复 spin_some 直到条件成立或超时。 */
    bool spinUntil(const std::function<bool()>& done,
                   std::chrono::milliseconds timeout) {
      const auto deadline = std::chrono::steady_clock::now() + timeout;
      while (std::chrono::steady_clock::now() < deadline) {
        executor_->spin_some();
        if (done()) {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      executor_->spin_some();
      return done();
    }

    /**
     * @brief 发布一帧（位姿 + 扫描）并等到该帧的 terrain_map 发布出来。
     * @return 收到新的 terrain_map 返回 true，超时返回 false。
     */
    bool publishFrame(double vehicle_x, double vehicle_y, double box_x,
                      double timestamp_sec) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = rclcpp::Time(
          static_cast<int64_t>(timestamp_sec * 1e9));
      odom.header.frame_id = "odom";
      odom.pose.pose.position.x = vehicle_x;
      odom.pose.pose.position.y = vehicle_y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.w = 1.0;
      pub_odom_->publish(odom);

      sensor_msgs::msg::PointCloud2 scan;
      pcl::toROSMsg(*makeFrame(vehicle_x, vehicle_y, box_x), scan);
      scan.header.stamp = odom.header.stamp;
      scan.header.frame_id = "odom";
      pub_scan_->publish(scan);

      const std::size_t before = map_count_;
      return spinUntil([this, before]() { return map_count_ > before; },
                       std::chrono::milliseconds(2000));
    }

    /** @brief 把最近收到的 terrain_map 转成 PCL 点云。 */
    pcl::PointCloud<pcl::PointXYZI> lastOutput() const {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      EXPECT_NE(last_map_, nullptr) << "尚未收到 terrain_map";
      if (last_map_ != nullptr) {
        pcl::fromROSMsg(*last_map_, cloud);
      }
      return cloud;
    }

    std::shared_ptr<TerrainAnalysis> terrain_;
    std::shared_ptr<rclcpp::Node> driver_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr last_map_;
    std::size_t map_count_ = 0;
  };

  // 只发布输出与时间戳接线：平坦地面不应产生任何障碍点，发布的消息必须带
  // 输入帧的时间戳与 odom 坐标系（下游 costmap 依赖这两项）。
  TEST_F(TerrainMapIntegrationTest,
         FlatGround_PublishesEmptyMapWithOdomHeader) {
    ASSERT_TRUE(publishFrame(0.0, 0.0, -1.0, 100.0));

    EXPECT_EQ(last_map_->header.frame_id, "odom");
    EXPECT_DOUBLE_EQ(rclcpp::Time(last_map_->header.stamp).seconds(), 100.0);

    const auto output = lastOutput();
    EXPECT_EQ(countNear(output, 0.0, 0.0, 5.0), 0U)
        << "平坦地面不应输出障碍点，实际输出 " << output.points.size() << " 点";
  }

  // 完整链路的高度语义：3 m 处方块顶面 0.35 m，输出点的 intensity 应等于它距
  // 局部地面的高度，而同一帧的地面点不应出现在输出里。
  TEST_F(TerrainMapIntegrationTest, BoxOnGround_OutputsHeightAsIntensity) {
    ASSERT_TRUE(publishFrame(0.0, 0.0, 3.0, 100.0));

    const auto output = lastOutput();
    EXPECT_GE(countNear(output, 3.0, 0.0, 0.3), 5U)
        << "方块处的输出点过少，实际 " << countNear(output, 3.0, 0.0, 0.3);

    const float height = maxIntensityNear(output, 3.0, 0.0, 0.3);
    EXPECT_GT(height, 0.20F);
    EXPECT_LT(height, 0.40F);

    EXPECT_EQ(countNear(output, 0.0, 0.0, 1.0), 0U) << "车下方的地面点不应输出";
  }

  // 幽灵点清除的集成版本：方块停止被观测后，输出中的残影应在有限帧内消失，
  // 不需要 dy_obs（该机制已删除，实测对清除延迟没有贡献）。
  TEST_F(TerrainMapIntegrationTest, RemovedBox_ClearsFromOutputWithinFrames) {
    constexpr double kBoxX = 4.2;
    double t = 100.0;
    for (int i = 0; i < 10; i++) {
      ASSERT_TRUE(publishFrame(0.0, 0.0, kBoxX, t))
          << "第 " << i << " 帧未发布";
      t += kFrameDt;
    }
    ASSERT_GT(countNear(lastOutput(), kBoxX, 0.0, 0.3), 0U)
        << "方块移除前就已不在输出里，本用例失去意义";

    int cleared_after = -1;
    for (int i = 0; i < 20 && cleared_after < 0; i++) {
      ASSERT_TRUE(publishFrame(0.0, 0.0, -1.0, t)) << "第 " << i << " 帧未发布";
      t += kFrameDt;
      if (countNear(lastOutput(), kBoxX, 0.0, 0.3) == 0) {
        cleared_after = i + 1;
      }
    }

    EXPECT_GT(cleared_after, 0) << "方块停止被观测后 20 帧仍未从输出中消失";
    RecordProperty("clear_latency_frames", cleared_after);
  }

  // 跨帧数据流的集成验证：车不动时，6 m 处的方块在采集窗口之外，不输出；
  // 车向前开到 4 m 后，体素地图里早就存下的这些点随窗口滚动进入采集范围，
  // 于是出现在输出里。这条链路只有驱动整个节点才会被跑到。
  TEST_F(TerrainMapIntegrationTest,
         DrivingForward_BringsStoredObstacleIntoOutput) {
    constexpr double kBoxX = 6.0;
    double t = 100.0;
    for (int i = 0; i < 5; i++) {
      ASSERT_TRUE(publishFrame(0.0, 0.0, kBoxX, t));
      t += kFrameDt;
    }
    EXPECT_EQ(countNear(lastOutput(), kBoxX, 0.0, 0.3), 0U)
        << "车在原地时 6 m 处已在采集窗口（±5.5 m）之外，不应输出";

    std::size_t seen = 0;
    for (int i = 0; i < 5 && seen == 0; i++) {
      ASSERT_TRUE(publishFrame(4.0, 0.0, kBoxX, t));
      t += kFrameDt;
      seen = countNear(lastOutput(), kBoxX, 0.0, 0.3);
    }
    EXPECT_GT(seen, 0U) << "车前进到 4 m 后，体素地图中已存的方块点应进入输出";
  }

}  // namespace terrain_analysis
