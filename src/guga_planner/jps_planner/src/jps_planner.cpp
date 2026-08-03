#include "jps_planner/jps_planner.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "bspline_opt/bspline_optimizer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rog_map_layer/esdf_layer.hpp"
#include "rog_map_layer/esdf_map.hpp"

namespace jps_planner {

  // ══════════════════════════════════════════════════════════════════════════════
  // configure — 读取 ROS 参数, 初始化状态
  // ══════════════════════════════════════════════════════════════════════════════

  void JPSPlanner::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("JPSPlanner: failed to lock parent node");
    }

    // 保存 ROS 基础设施的引用
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();  // 获取底层 Costmap2D 原始指针
    global_frame_ = costmap_ros_->getGlobalFrameID();
    clock_ = node->get_clock();
    logger_ = node->get_logger();

    // ── JPS 搜索参数 ──
    // w_traversal_cost: 单格元通行代价权重 (Theta* 缩放后)
    // w_euc_cost:       跳转点间欧几里得距离代价权重
    // w_heuristic_cost: A* 启发式权重 (控制贪心程度)
    // allow_unknown:    是否允许穿越未知空间 (255 = NO_INFORMATION)
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(10.0));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".w_euc_cost",
                                                 rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".w_heuristic_cost", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".allow_unknown",
                                                 rclcpp::ParameterValue(false));

    // B-spline 平滑开关
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".enable_bspline", rclcpp::ParameterValue(true));

    node->get_parameter(name_ + ".w_traversal_cost", config_.w_traversal_cost);
    node->get_parameter(name_ + ".w_euc_cost", config_.w_euc_cost);
    node->get_parameter(name_ + ".w_heuristic_cost", config_.w_heuristic_cost);
    node->get_parameter(name_ + ".allow_unknown", config_.allow_unknown);
    node->get_parameter(name_ + ".enable_bspline", enable_bspline_);

    // ── ESDF 梯度优化参数 ──
    // enable_esdf:        是否启用 ESDF 辅助优化
    // esdf_weight:        ESDF 距离代价权重 w_e
    // esdf_safe_distance: 安全距离 d_safe (米),
    //                     路径离障碍物小于此距离时开始产生惩罚
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".enable_esdf",
                                                 rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".esdf_weight",
                                                 rclcpp::ParameterValue(100.0));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".esdf_safe_distance", rclcpp::ParameterValue(0.3));

    node->get_parameter(name_ + ".enable_esdf", enable_esdf_);
    node->get_parameter(name_ + ".esdf_weight", esdf_weight_);
    node->get_parameter(name_ + ".esdf_safe_distance", esdf_safe_distance_);

    RCLCPP_INFO(
        logger_,
        "JPSPlanner configured: w_traversal=%.2f w_euc=%.2f "
        "w_heuristic=%.2f allow_unknown=%d enable_bspline=%d enable_esdf=%d",
        config_.w_traversal_cost, config_.w_euc_cost, config_.w_heuristic_cost,
        config_.allow_unknown, enable_bspline_, enable_esdf_);

    // 初始化共享内存写入端 — 将规划结果推送给 Pangolin UI 渲染
    shm_ready_ = shm_writer_.init("guga_shm", guga_ui::UiSlotId::PATH);
    if (!shm_ready_) {
      RCLCPP_ERROR(logger_,
                   "ShmWriter init failed, UI path display unavailable");
    } else {
      RCLCPP_INFO(logger_, "ShmWriter initialized for UI path display");
    }
  }

  void JPSPlanner::cleanup() {
    RCLCPP_INFO(logger_, "JPSPlanner: cleaning up");
    costmap_ = nullptr;
    costmap_ros_.reset();
    tf_.reset();
    is_active_ = false;
  }

  void JPSPlanner::activate() {
    RCLCPP_INFO(logger_, "JPSPlanner: activating");
    is_active_ = true;
  }

  void JPSPlanner::deactivate() {
    RCLCPP_INFO(logger_, "JPSPlanner: deactivating");
    is_active_ = false;
  }

  // ══════════════════════════════════════════════════════════════════════════════
  // createPlan — JPS 搜索 + B-spline 平滑 + ESDF 梯度优化
  //
  // 完整数据流:
  //   world (start/goal)
  //     │
  //     ▼ costmap_->worldToMap()
  //   cell coords (sx,sy) → (gx,gy)
  //     │
  //     ▼ JPSAlgorithm::generatePath()
  //   map path (跳转点, 格元中心)
  //     │
  //     ▼ bsplineSmooth()
  //     │  ├ BSplineOptimizer::fit()    — chord-length 参数化, 精确插值
  //     │  ├ ESDF 注入                   — 从 LayeredCostmap 查找 EsdfLayer
  //     │  └ BSplineOptimizer::optimize() — 梯度下降 (可选) + 障碍物投射
  //     │
  //     ▼ costmap_->mapToWorld()
  //   world path (nav_msgs::Path)
  // ══════════════════════════════════════════════════════════════════════════════

  nav_msgs::msg::Path JPSPlanner::createPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal) {
    nav_msgs::msg::Path plan;
    plan.header.stamp = clock_->now();
    plan.header.frame_id = global_frame_;

    if (!is_active_) {
      RCLCPP_WARN(logger_, "JPSPlanner: not active, returning empty plan");
      return plan;
    }

    // 验证代价地图可用
    if (costmap_ == nullptr) {
      RCLCPP_ERROR(logger_, "JPSPlanner: costmap is null");
      return plan;
    }

    // 将起点/终点从世界坐标转换到代价地图格元坐标
    unsigned int mx_start{}, my_start{};
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                              mx_start, my_start)) {
      RCLCPP_ERROR(logger_,
                   "JPSPlanner: start pose (%.2f, %.2f) out of costmap bounds",
                   start.pose.position.x, start.pose.position.y);
      return plan;
    }

    unsigned int mx_goal{}, my_goal{};
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y,
                              mx_goal, my_goal)) {
      RCLCPP_ERROR(logger_,
                   "JPSPlanner: goal pose (%.2f, %.2f) out of costmap bounds",
                   goal.pose.position.x, goal.pose.position.y);
      return plan;
    }

    int sx = static_cast<int>(mx_start);
    int sy = static_cast<int>(my_start);
    int gx = static_cast<int>(mx_goal);
    int gy = static_cast<int>(my_goal);

    RCLCPP_INFO(logger_,
                "JPSPlanner: planning from (%d, %d) to (%d, %d) [cells]", sx,
                sy, gx, gy);

    // ── 第 1 步: JPS 搜索 (模式 A 函数式数据流) ──
    // state 持有 costmap 指针和 A* 展开所需的数据结构
    JPSState state{};
    state.costmap_data = costmap_->getCharMap();
    state.size_x = static_cast<int>(costmap_->getSizeInCellsX());
    state.size_y = static_cast<int>(costmap_->getSizeInCellsY());

    std::vector<std::pair<double, double>> map_path{};
    bool found = JPSAlgorithm::generatePath(config_, state, sx, sy, gx, gy,
                                            map_path);

    if (!found) {
      RCLCPP_WARN(logger_, "JPSPlanner: no path found");
      return plan;
    }

    RCLCPP_INFO(logger_, "JPSPlanner: path found with %zu waypoints",
                map_path.size());

    // ── 第 2 步: B-spline 平滑 + (可选) ESDF 梯度优化 ──
    // 需要 ≥ 8 个航点才能使用完整 7 阶 B-spline
    if (enable_bspline_ && map_path.size() >= 8) {
      plan = bsplineSmooth(map_path, costmap_->getCharMap(),
                           static_cast<int>(costmap_->getSizeInCellsX()),
                           static_cast<int>(costmap_->getSizeInCellsY()),
                           costmap_->getResolution());
      RCLCPP_INFO(logger_, "JPSPlanner: B-spline smooth applied, %zu poses",
                  plan.poses.size());
    } else {
      // 路径点太少或 B-spline 被禁用 → 线性插值
      if (enable_bspline_) {
        RCLCPP_INFO(
            logger_,
            "JPSPlanner: too few waypoints (%zu) for B-spline, linear fallback",
            map_path.size());
      }
      std::vector<std::pair<double, double>> world_path{};
      world_path.reserve(map_path.size());
      for (const auto& [mx, my] : map_path) {
        double wx{}, wy{};
        costmap_->mapToWorld(static_cast<unsigned int>(mx),
                             static_cast<unsigned int>(my), wx, wy);
        world_path.emplace_back(wx, wy);
      }
      plan = linearInterpolation(world_path, costmap_->getResolution());
    }

    plan.header.stamp = clock_->now();
    plan.header.frame_id = global_frame_;

    // 推送路径到共享内存供 UI 渲染
    writePathToShm(plan);

    return plan;
  }

  // ══════════════════════════════════════════════════════════════════════════════
  // bsplineSmooth — B-spline 拟合 + 可选的 ESDF 梯度优化
  //
  // 数学原理:
  //   代价函数 J(P) = w_s·J_smooth + w_d·J_dist + w_o·J_obs + w_e·J_esdf
  //
  //   其中各分量分别为:
  //     J_smooth = ∫‖C''‖² du          — 曲率能量, 保持路径平滑
  //     J_dist   = Σ‖C(τ_i)-q_i‖²     — 偏离原始 JPS 航点的距离
  //     J_obs    = Σ δ_i·w_o·(1+‖p_i-c_i‖) — 二元障碍物惩罚 (cost≥253)
  //     J_esdf   = Σ [max(0, d_safe - d_esdf(p_i))]² — 连续距离场避障
  //
  //   ESDF (Euclidean Signed Distance Field) 提供每个格元到最近障碍物的
  //   精确欧几里得距离。通过双线性插值在连续坐标上查询, 产生平滑的梯度。
  //   代价随距离减小而二次增长, 将路径推向远离障碍物的方向。
  //
  //   ESDF 代价 J_esdf 仅在启用 enable_esdf 时计算,
  //   此时会自动开启梯度下降 (enable_gradient_descent=true)。
  // ══════════════════════════════════════════════════════════════════════════════

  nav_msgs::msg::Path JPSPlanner::bsplineSmooth(
      const std::vector<std::pair<double, double>>& map_path,
      const unsigned char* costmap_data, int cm_w, int cm_h,
      double resolution) {
    nav_msgs::msg::Path plan;

    // ── 第 1 步: 创建 B-spline 优化器并拟合 JPS 路径 ──
    // fit() 使用 chord-length 参数化 + Eigen SplineFitting::Interpolate,
    // 将 JPS 航点精确插值为 7 阶 C2 连续 B-spline 曲线
    bspline_opt::BSplineOptimizer opt(bspline_config_);
    if (!opt.fit(map_path)) {
      RCLCPP_WARN(
          logger_,
          "JPSPlanner: B-spline fit failed, using linear interpolation");
      std::vector<std::pair<double, double>> world_path{};
      world_path.reserve(map_path.size());
      for (const auto& [mx, my] : map_path) {
        double wx{}, wy{};
        costmap_->mapToWorld(static_cast<unsigned int>(mx),
                             static_cast<unsigned int>(my), wx, wy);
        world_path.emplace_back(wx, wy);
      }
      return linearInterpolation(world_path, resolution);
    }

    // 注入代价地图 — 用于二元障碍物检查 (cost ≥ 253 = 障碍物)
    opt.state().costmap_data = costmap_data;
    opt.state().costmap_w = cm_w;
    opt.state().costmap_h = cm_h;

    // ════════════════════════════════════════════════════════════════════════
    // 第 2 步: 注入 ESDF 数据 (启用时)
    //
    // 通过 costmap_ros_->getLayeredCostmap() → getPlugins() 查找 EsdfLayer,
    // 获取其内部 EsdfMap 的距离场和梯度场数据指针。
    //
    // ESDF 距离场: esdf_distance[i] = 格元 i 到最近障碍物的距离 (米)
    // 梯度场:     esdf_gradient_x/y[i] = ∂d/∂x, ∂d/∂y (距离变化/格元)
    //
    // 在 gradientDescent 中, evalCost 通过对 ESDF 距离场做双线性插值,
    // 计算 J_esdf = w_e * Σ [max(0, d_safe - d_esdf(p))]²,
    // 产生平滑连续的避障梯度。
    // ════════════════════════════════════════════════════════════════════════
    if (enable_esdf_) {
      auto* layered_costmap = costmap_ros_->getLayeredCostmap();
      if (layered_costmap) {
        auto plugins = layered_costmap->getPlugins();
        if (plugins) {
          for (auto& plugin : *plugins) {
            auto esdf_layer =
                std::dynamic_pointer_cast<rog_map_layer::EsdfLayer>(plugin);
            if (esdf_layer) {
              const auto* esdf_map = esdf_layer->getEsdfMapRaw();
              if (esdf_map) {
                const auto& esdf_cfg = esdf_layer->config();
                // 将 ESDF 数据指针注入 BSplineState
                // 指针非拥有 — BSplineOptimizer 不负责释放
                opt.state().esdf_distance = esdf_map->distanceField().data();
                opt.state().esdf_gradient_x = esdf_map->gradientX().data();
                opt.state().esdf_gradient_y = esdf_map->gradientY().data();
                opt.state().esdf_w = static_cast<int>(esdf_map->sizeX());
                opt.state().esdf_h = static_cast<int>(esdf_map->sizeY());
                opt.state().esdf_resolution = esdf_map->resolution();
                opt.state().esdf_origin_x = esdf_map->originX();
                opt.state().esdf_origin_y = esdf_map->originY();
                opt.state().esdf_max_distance = esdf_cfg.max_distance;

                // 启用 ESDF 模式并自动开启梯度下降
                bspline_config_.enable_esdf = true;
                bspline_config_.enable_gradient_descent = true;
                bspline_config_.esdf_weight = esdf_weight_;
                bspline_config_.esdf_safe_distance = esdf_safe_distance_;

                RCLCPP_INFO(
                    logger_,
                    "JPSPlanner: ESDF layer found, enabling gradient descent "
                    "with w_esdf=%.1f safe_dist=%.2f",
                    esdf_weight_, esdf_safe_distance_);
              }
              break;
            }
          }
        }
      }
      if (!bspline_config_.enable_esdf) {
        RCLCPP_WARN(logger_,
                    "JPSPlanner: ESDF enabled but EsdfLayer not found in "
                    "costmap plugins, "
                    "falling back to binary obstacle avoidance.");
      }
    }

    // ════════════════════════════════════════════════════════════════════════
    // 第 3 步: 运行优化
    //
    // (a) 梯度下降 (仅 enabled):
    //     变量 = 内部控制点 [2*(M-2) 维]
    //     方法 = 数值中心差分 (h=0.5 格元) + 回溯线搜索
    //     约束 = |x_i - x_init| ≤ corridor_halfwidth (走廊约束)
    //
    // (b) 障碍物投射 (始终执行):
    //     螺旋搜索 (半径 8 格元) 将落在障碍物内的控制点投射到最近空闲格元
    //     对采样路径点逐一同样操作
    //
    // (c) 输出:
    //     smoothed_path (N 个 (x,y) 点)
    //     curvature_profile (每个点的曲率 κ)
    // ════════════════════════════════════════════════════════════════════════
    int num_samples = std::max(100, static_cast<int>(map_path.size()) * 5);
    auto result = opt.optimize(num_samples);

    // ── 第 4 步: 地图坐标 → 世界坐标 ──
    plan.poses.reserve(result.smoothed_path.size());
    for (size_t i = 0; i < result.smoothed_path.size(); ++i) {
      double wx{}, wy{};
      costmap_->mapToWorld(
          static_cast<unsigned int>(result.smoothed_path[i].first),
          static_cast<unsigned int>(result.smoothed_path[i].second), wx, wy);
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      // 朝向角: atan2(Δy, Δx), 指向下一个航点
      double yaw{};
      if (i + 1 < result.smoothed_path.size()) {
        double dx = result.smoothed_path[i + 1].first
                    - result.smoothed_path[i].first;
        double dy = result.smoothed_path[i + 1].second
                    - result.smoothed_path[i].second;
        yaw = std::atan2(dy, dx);
      }
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
          yaw);
      plan.poses.push_back(pose);
    }

    return plan;
  }

  nav_msgs::msg::Path JPSPlanner::linearInterpolation(
      const std::vector<std::pair<double, double>>& raw_path,
      double resolution) {
    nav_msgs::msg::Path plan;
    if (raw_path.empty()) {
      return plan;
    }
    if (raw_path.size() == 1) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = raw_path[0].first;
      pose.pose.position.y = raw_path[0].second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.poses.push_back(pose);
      return plan;
    }

    plan.poses.reserve(raw_path.size() * 2);

    for (size_t i = 0; i < raw_path.size() - 1; ++i) {
      double x0 = raw_path[i].first;
      double y0 = raw_path[i].second;
      double x1 = raw_path[i + 1].first;
      double y1 = raw_path[i + 1].second;

      double dist = std::hypot(x1 - x0, y1 - y0);
      int steps = std::max(1, static_cast<int>(std::ceil(dist / resolution)));

      for (int s = 0; s < steps; ++s) {
        double t = static_cast<double>(s) / static_cast<double>(steps);
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x0 + t * (x1 - x0);
        pose.pose.position.y = y0 + t * (y1 - y0);
        pose.pose.position.z = 0.0;
        // 简单的朝向: 指向下一个航点
        double yaw = std::atan2(y1 - y0, x1 - x0);
        pose.pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(yaw);
        plan.poses.push_back(pose);
      }
    }

    // 添加最后一个点
    geometry_msgs::msg::PoseStamped final_pose;
    final_pose.pose.position.x = raw_path.back().first;
    final_pose.pose.position.y = raw_path.back().second;
    final_pose.pose.position.z = 0.0;
    final_pose.pose.orientation = plan.poses.back().pose.orientation;
    plan.poses.push_back(final_pose);

    return plan;
  }

  void JPSPlanner::writePathToShm(const nav_msgs::msg::Path& plan) {
    if (!shm_ready_) {
      return;
    }

    const size_t n = plan.poses.size();
    if (n == 0) {
      return;
    }

    guga_ui::UiPath ui_path{};
    ui_path.stamp_sec = clock_->now().seconds();

    // 降采样: 每 stride 个点取 1 个 (目标 ≤ UI_PATH_MAX_POINTS)
    size_t stride = (n <= guga_ui::UI_PATH_MAX_POINTS)
                        ? 1
                        : (n / guga_ui::UI_PATH_MAX_POINTS + 1);
    ui_path.count = static_cast<uint32_t>((n + stride - 1) / stride);
    if (ui_path.count > guga_ui::UI_PATH_MAX_POINTS) {
      ui_path.count = guga_ui::UI_PATH_MAX_POINTS;
    }

    for (uint32_t i = 0; i < ui_path.count; ++i) {
      size_t src = i * stride;
      ui_path.x[i] = plan.poses[src].pose.position.x;
      ui_path.y[i] = plan.poses[src].pose.position.y;
    }

    shm_writer_.write(&ui_path, sizeof(ui_path));
  }

}  // namespace jps_planner

PLUGINLIB_EXPORT_CLASS(jps_planner::JPSPlanner, nav2_core::GlobalPlanner)
