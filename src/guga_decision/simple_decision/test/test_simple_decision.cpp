#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "simple_decision/node/simple_decision.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "test_helpers.hpp"

namespace simple_decision {
  using namespace std::chrono_literals;

  class DecisionSimpleTest : public testing::Test {
  public:
    ~DecisionSimpleTest() override = default;
    DecisionSimpleTest(const DecisionSimpleTest&) = delete;
    DecisionSimpleTest& operator=(const DecisionSimpleTest&) = delete;
    DecisionSimpleTest(DecisionSimpleTest&&) = delete;
    DecisionSimpleTest& operator=(DecisionSimpleTest&&) = delete;

  protected:
    struct SystemConfig {
      bool require_game_running = false;
      double start_delay_sec = 5.0;
      double default_goal_hz = 2.0;
      double supply_goal_hz = 2.0;
      int hp_survival_enter = 120;
      int hp_survival_exit = 300;
      int ammo_min = 0;
    };

    DecisionSimpleTest() {
      CreateSystem(SystemConfig{});
    }

    void TearDown() override {
      ASSERT_NO_THROW(DestroySystem());
    }

    void CreateSystem(const SystemConfig& sys) {
      rclcpp::NodeOptions opts;
      opts.append_parameter_override("require_game_running",
                                     sys.require_game_running);
      opts.append_parameter_override("start_delay_sec", sys.start_delay_sec);
      opts.append_parameter_override("default_goal_hz", sys.default_goal_hz);
      opts.append_parameter_override("supply_goal_hz", sys.supply_goal_hz);
      opts.append_parameter_override("hp_survival_enter",
                                     sys.hp_survival_enter);
      opts.append_parameter_override("hp_survival_exit", sys.hp_survival_exit);
      opts.append_parameter_override("ammo_min", sys.ammo_min);

      node_ = std::make_shared<DecisionSimple>(opts);
      helper_ = std::make_shared<rclcpp::Node>("test_helper");

      game_status_pub_ =
          helper_->create_publisher<guga_interfaces::msg::GameStatus>(
              "referee/game_status", rclcpp::QoS(10));
      robot_status_pub_ =
          helper_->create_publisher<guga_interfaces::msg::RobotStatus>(
              "referee/robot_status", rclcpp::QoS(10));
      armor_pub_ = helper_->create_publisher<guga_interfaces::msg::Armors>(
          "detector/armors", rclcpp::QoS(10));
      target_pub_ = helper_->create_publisher<guga_interfaces::msg::Target>(
          "tracker/target", rclcpp::QoS(10));

      chassis_mode_sub_ = helper_->create_subscription<std_msgs::msg::UInt8>(
          "chassis_mode", rclcpp::QoS(10),
          [this](const std_msgs::msg::UInt8::SharedPtr m) {
            chassis_modes_.push_back(m->data);
          });
      goal_pose_sub_ =
          helper_->create_subscription<geometry_msgs::msg::PoseStamped>(
              "goal_pose", rclcpp::SensorDataQoS(),
              [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
                goal_poses_.push_back(*m);
              });
      debug_attack_sub_ =
          helper_->create_subscription<geometry_msgs::msg::PoseStamped>(
              "debug_attack_pose", rclcpp::QoS(10),
              [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
                debug_attack_poses_.push_back(*m);
              });

      exec_.add_node(node_);
      exec_.add_node(helper_);
      WaitForConnections();
    }

    void WaitForConnections() {
      WaitUntil([this]() { return AllConnected(); }, 1000ms);
    }

    bool AllConnected() {
      return game_status_pub_->get_subscription_count() > 0
          && robot_status_pub_->get_subscription_count() > 0
          && armor_pub_->get_subscription_count() > 0
          && target_pub_->get_subscription_count() > 0
          && chassis_mode_sub_->get_publisher_count() > 0
          && goal_pose_sub_->get_publisher_count() > 0
          && debug_attack_sub_->get_publisher_count() > 0;
    }

    template <typename PublisherT, typename MsgT>
    void PublishAndSpin(const std::shared_ptr<PublisherT>& pub,
                        const MsgT& msg) {
      pub->publish(msg);
      exec_.spin_some();
    }

    void SendRobotStatus(uint16_t hp, uint16_t ammo,
                         bool is_hp_deduced = false) {
      auto msg = guga_interfaces::msg::RobotStatus();
      msg.current_hp = hp;
      msg.projectile_allowance_17mm = ammo;
      msg.is_hp_deduced = is_hp_deduced;
      PublishAndSpin(robot_status_pub_, msg);
    }

    void SendGameStatus(uint8_t progress) {
      auto msg = guga_interfaces::msg::GameStatus();
      msg.game_progress = progress;
      PublishAndSpin(game_status_pub_, msg);
    }

    void SendArmors(const std::vector<std::array<double, 3>>& positions) {
      auto msg = guga_interfaces::msg::Armors();
      for (const auto& p : positions) {
        guga_interfaces::msg::Armor a;
        a.pose.position.x = p[0];
        a.pose.position.y = p[1];
        a.pose.position.z = p[2];
        msg.armors.push_back(a);
      }
      PublishAndSpin(armor_pub_, msg);
    }

    void SendTarget(double x, double y, double z, double yaw, bool tracking) {
      auto msg = guga_interfaces::msg::Target();
      msg.position.x = x;
      msg.position.y = y;
      msg.position.z = z;
      msg.yaw = yaw;
      msg.tracking = tracking;
      PublishAndSpin(target_pub_, msg);
    }

    void SpinFor(std::chrono::milliseconds ms) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < ms) {
        exec_.spin_some();
        std::this_thread::sleep_for(10ms);
      }
    }

    template <typename Predicate>
    bool WaitUntil(Predicate&& pred, std::chrono::milliseconds timeout) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < timeout) {
        exec_.spin_some();
        if (pred()) {
          return true;
        }
        std::this_thread::sleep_for(10ms);
      }
      return false;
    }

    bool WaitForChassisAtLeast(size_t n, std::chrono::milliseconds timeout) {
      return WaitUntil([this, n]() { return chassis_modes_.size() >= n; },
                       timeout);
    }

    bool WaitForGoalAtLeast(size_t n, std::chrono::milliseconds timeout) {
      return WaitUntil([this, n]() { return goal_poses_.size() >= n; },
                       timeout);
    }

    void ClearReceived() {
      chassis_modes_.clear();
      goal_poses_.clear();
      debug_attack_poses_.clear();
    }

    void CallHandleGateLog(Readiness& r) {
      node_->handleGateLog(r);
    }

    std::optional<Pose2D> CallGetRobotPoseMap() {
      return node_->getRobotPoseMap();
    }

    void DestroySystem() {
      ClearReceived();
      exec_.remove_node(helper_);
      exec_.remove_node(node_);
      game_status_pub_.reset();
      robot_status_pub_.reset();
      armor_pub_.reset();
      target_pub_.reset();
      chassis_mode_sub_.reset();
      goal_pose_sub_.reset();
      debug_attack_sub_.reset();
      helper_.reset();
      node_.reset();
    }

    void PublishStaticTF(const std::string& parent, const std::string& child,
                         double x, double y, double yaw) {
      auto bc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = helper_->now();
      t.header.frame_id = parent;
      t.child_frame_id = child;
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      bc->sendTransform(t);
      SpinFor(200ms);
    }

    rclcpp::executors::SingleThreadedExecutor exec_;
    std::shared_ptr<DecisionSimple> node_;
    rclcpp::Node::SharedPtr helper_;

    rclcpp::Publisher<guga_interfaces::msg::GameStatus>::SharedPtr
        game_status_pub_;
    rclcpp::Publisher<guga_interfaces::msg::RobotStatus>::SharedPtr
        robot_status_pub_;
    rclcpp::Publisher<guga_interfaces::msg::Armors>::SharedPtr armor_pub_;
    rclcpp::Publisher<guga_interfaces::msg::Target>::SharedPtr target_pub_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        debug_attack_sub_;

    std::vector<uint8_t> chassis_modes_;
    std::vector<geometry_msgs::msg::PoseStamped> goal_poses_;
    std::vector<geometry_msgs::msg::PoseStamped> debug_attack_poses_;
  };

  // ------ Gate --------------------------------------------------------
  // 没有 robot_status 时节点不发任何 goal_pose，避免基于过期数据做决策

  TEST_F(DecisionSimpleTest, Tick_NoRobotStatus_NoOutput) {
    size_t initial = chassis_modes_.size();
    SendGameStatus(GameStatus::RUNNING);
    SpinFor(300ms);
    EXPECT_EQ(chassis_modes_.size(), initial);
    EXPECT_EQ(goal_poses_.size(), 0u);
  }

  // 有 robot_status 且 require_game_running=false 时立刻发 goal
  TEST_F(DecisionSimpleTest, Tick_WithRobotStatus_PublishesGoal) {
    SendRobotStatus(500, 120);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));
  }

  // 门控组合：require_game + start_delay + RS/GS 状态 → 是否发 goal
  struct GateTestParam {
    bool require_game_running;
    double start_delay_sec;
    bool send_robot;
    bool send_game;
    uint8_t game_progress;
    std::chrono::milliseconds wait;
    bool expect_goal;
  };

  class DecisionSimpleGateTest
      : public DecisionSimpleTest,
        public testing::WithParamInterface<GateTestParam> {
  protected:
    DecisionSimpleGateTest() {
      const auto& p = GetParam();
      DestroySystem();
      CreateSystem({p.require_game_running, p.start_delay_sec});
      ClearReceived();
    }
  };

  INSTANTIATE_TEST_SUITE_P(
      Gates, DecisionSimpleGateTest,
      testing::Values(
          GateTestParam{true, 5.0, false, true,
                        guga_interfaces::msg::GameStatus::RUNNING, 500ms, false},
          GateTestParam{true, 5.0, true, false,
                        guga_interfaces::msg::GameStatus::NOT_START, 500ms,
                        false},
          GateTestParam{true, 5.0, true, true,
                        guga_interfaces::msg::GameStatus::COUNT_DOWN, 600ms,
                        false},
          GateTestParam{true, 5.0, true, true,
                        guga_interfaces::msg::GameStatus::RUNNING, 300ms, false},
          GateTestParam{true, 0.1, true, true,
                        guga_interfaces::msg::GameStatus::RUNNING, 1400ms,
                        true}));

  // 五个场景：无RS→不发 / 无GS→不发 / 未开始→不发 / delay内→不发 / delay后→发
  TEST_P(DecisionSimpleGateTest, GateBehavior) {
    const auto& p = GetParam();
    if (p.send_robot) {
      SendRobotStatus(500, 120);
    }
    if (p.send_game) {
      SendGameStatus(p.game_progress);
    }
    SpinFor(p.wait);

    if (!p.expect_goal) {
      EXPECT_EQ(goal_poses_.size(), 0u);
      return;
    }
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));
  }

  // 比赛离开 RUNNING 状态后门控复位，不再发 goal
  class DecisionSimpleRunningGateTest : public DecisionSimpleTest {
  protected:
    DecisionSimpleRunningGateTest() {
      DestroySystem();
      CreateSystem({true, 0.0});
      ClearReceived();
    }
  };

  TEST_F(DecisionSimpleRunningGateTest,
         LeaveRunning_ResetsGate_NoFurtherOutput) {
    SendRobotStatus(500, 120);
    SendGameStatus(guga_interfaces::msg::GameStatus::RUNNING);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));

    ClearReceived();
    SendGameStatus(guga_interfaces::msg::GameStatus::GAME_OVER);
    SpinFor(500ms);

    EXPECT_EQ(chassis_modes_.size(), 0u);
    EXPECT_EQ(goal_poses_.size(), 0u);
  }

  // 四种门控状态对应的 warn 日志均不崩溃，READY 走 default 分支结束
  TEST_F(DecisionSimpleTest, HandleGateLog_AllStatuses_DoNotCrash) {
    Readiness r;
    r.status = Readiness::Status::NO_RS;
    CallHandleGateLog(r);
    r.status = Readiness::Status::NO_GS;
    CallHandleGateLog(r);
    r.status = Readiness::Status::NOT_STARTED;
    CallHandleGateLog(r);
    r.status = Readiness::Status::IN_DELAY;
    r.elapsed = 1.5;
    CallHandleGateLog(r);
    r.status = Readiness::Status::READY;
    CallHandleGateLog(r);
    SUCCEED();
  }

  // ------ Decision ----------------------------------------------------

  // hp 低于补给阈值 → 进入 SUPPLY 态，target 坐标指向补给点
  TEST_F(DecisionSimpleTest, LowHp_EntersSupply_PublishesSupplyGoal) {
    SendRobotStatus(100, 120);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));
    const auto& g = goal_poses_.back();
    EXPECT_NEAR(g.pose.position.x, kSupplyX, 1e-6);
    EXPECT_NEAR(g.pose.position.y, kSupplyY, 1e-6);
  }

  // hp/ammo 恢复到阈值以上 → 从 SUPPLY 退出到 DEFAULT，target 坐标切回默认点
  TEST_F(DecisionSimpleTest, SupplyRecovered_ExitsToDefaultGoal) {
    SendRobotStatus(100, 120);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));

    ClearReceived();
    SendRobotStatus(350, 120);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));
    const auto& g = goal_poses_.back();
    EXPECT_NEAR(g.pose.position.x, kDefaultX, 1e-6);
    EXPECT_NEAR(g.pose.position.y, kDefaultY, 1e-6);
  }

  // 受击后 chassis 切为 LITTLE_TES，增加闪避
  TEST_F(DecisionSimpleTest, AttackedRecent_UsesLittleTesMode) {
    SendRobotStatus(500, 120, true);
    ASSERT_TRUE(WaitForChassisAtLeast(2, 2000ms));
    EXPECT_EQ(chassis_modes_.back(),
              static_cast<uint8_t>(ChassisMode::LITTLE_TES));
  }

  // attacked_recent hold 过期后 chassis 从 LITTLE_TES 回到 CHASSIS_FOLLOWED
  TEST_F(DecisionSimpleTest, AttackedHoldExpired_BackToFollowed) {
    SendRobotStatus(500, 120, true);
    ASSERT_TRUE(WaitForChassisAtLeast(2, 2000ms));
    EXPECT_EQ(chassis_modes_.back(),
              static_cast<uint8_t>(ChassisMode::LITTLE_TES));

    ClearReceived();
    SendRobotStatus(500, 120, false);
    SpinFor(2000ms);

    ASSERT_TRUE(WaitForChassisAtLeast(1, 2000ms));
    EXPECT_EQ(chassis_modes_.back(),
              static_cast<uint8_t>(ChassisMode::CHASSIS_FOLLOWED));
  }

  // ------ Rate-limiting -----------------------------------------------

  // 默认态 goal_pose 发布按 default_goal_hz=1.0 节流
  class DecisionSimpleGoalHzTest : public DecisionSimpleTest {
  protected:
    DecisionSimpleGoalHzTest() {
      DestroySystem();
      CreateSystem({false, 5.0, 1.0});
      ClearReceived();
    }
  };

  TEST_F(DecisionSimpleGoalHzTest, DefaultGoalPublishing_ThrottledByHz) {
    SendRobotStatus(500, 120);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));
    size_t count = goal_poses_.size();

    SpinFor(300ms);
    EXPECT_EQ(goal_poses_.size(), count) << "within throttle window";

    SpinFor(900ms);
    EXPECT_GE(goal_poses_.size(), count + 1) << "past throttle window";
  }

  // SUPPLY 态 goal_pose 按 supply_goal_hz=1.0 节流
  class DecisionSimpleSupplyHzTest : public DecisionSimpleTest {
  protected:
    DecisionSimpleSupplyHzTest() {
      DestroySystem();
      CreateSystem({false, 5.0, 2.0, 1.0});
      ClearReceived();
    }
  };

  TEST_F(DecisionSimpleSupplyHzTest, SupplyGoalPublishing_ThrottledByHz) {
    SendRobotStatus(50, 120);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms));
    size_t count = goal_poses_.size();

    SpinFor(300ms);
    EXPECT_EQ(goal_poses_.size(), count) << "within throttle window";

    SpinFor(900ms);
    EXPECT_GE(goal_poses_.size(), count + 1) << "past throttle window";
  }

  // ------ Helpers -----------------------------------------------------

  // TF 存在时返回 Pose2D，x/y/yaw 与静态 TF 一致
  TEST_F(DecisionSimpleTest, GetRobotPoseMap_TfExists_ReturnsPose) {
    PublishStaticTF("map", "base_footprint", 1.0, 2.0, 0.0);

    auto result = CallGetRobotPoseMap();
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->x, 1.0);
    EXPECT_DOUBLE_EQ(result->y, 2.0);
    EXPECT_DOUBLE_EQ(result->yaw, 0.0);
  }

  // 没有静态 TF 时返回 nullopt，不崩溃
  TEST_F(DecisionSimpleTest, GetRobotPoseMap_NoTf_ReturnsNullopt) {
    auto result = CallGetRobotPoseMap();
    EXPECT_FALSE(result.has_value());
  }

}  // namespace simple_decision

class Ros2GlobalEnvironment : public ::testing::Environment {
public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }
  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

::testing::Environment* const kRos2Env = ::testing::AddGlobalTestEnvironment(
    new Ros2GlobalEnvironment());
