#include "jps_planner/jps_planner.hpp"

#include <algorithm>
#include <cmath>
#include <guga_ui_common/ui_types.hpp>
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

  namespace {

    constexpr unsigned char UNKNOWN_COST = 255;
    constexpr size_t MIN_BSPLINE_WAYPOINTS = 8;
    constexpr double BSPLINE_DENSIFY_STEP_CELLS = 2.0;
    constexpr double MIN_DENSIFY_STEP_CELLS = 0.5;

    [[nodiscard]] double pathLengthCells(
        const std::vector<std::pair<double, double>>& path) {
      double length{};
      for (size_t i = 1; i < path.size(); ++i) {
        length += std::hypot(path[i].first - path[i - 1].first,
                             path[i].second - path[i - 1].second);
      }
      return length;
    }

    [[nodiscard]] std::vector<std::pair<double, double>> densifyMapPath(
        const std::vector<std::pair<double, double>>& path,
        double max_step_cells, size_t target_min_points) {
      if (path.size() < 2) {
        return path;
      }

      const double total_length = pathLengthCells(path);
      if (total_length < 1e-9) {
        return path;
      }

      double step = max_step_cells;
      if (target_min_points > path.size()) {
        step = std::min(
            step, total_length / static_cast<double>(target_min_points - 1));
      }
      step = std::max(step, MIN_DENSIFY_STEP_CELLS);

      std::vector<std::pair<double, double>> dense_path{};
      dense_path.reserve(
          std::max(path.size(),
                   static_cast<size_t>(std::ceil(total_length / step)) + 1));
      dense_path.push_back(path.front());

      for (size_t i = 1; i < path.size(); ++i) {
        const double x0 = path[i - 1].first;
        const double y0 = path[i - 1].second;
        const double x1 = path[i].first;
        const double y1 = path[i].second;
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double segment_length = std::hypot(dx, dy);
        if (segment_length < 1e-9) {
          continue;
        }

        const int steps = std::max(
            1, static_cast<int>(std::ceil(segment_length / step)));
        for (int s = 1; s <= steps; ++s) {
          const double t = static_cast<double>(s) / static_cast<double>(steps);
          dense_path.emplace_back(x0 + (t * dx), y0 + (t * dy));
        }
      }

      return dense_path;
    }

    [[nodiscard]] std::pair<double, double> mapContinuousToWorld(
        const nav2_costmap_2d::Costmap2D& costmap, double mx, double my) {
      return {costmap.getOriginX() + (mx * costmap.getResolution()),
              costmap.getOriginY() + (my * costmap.getResolution())};
    }

    [[nodiscard]] std::vector<std::pair<double, double>> mapPathToWorld(
        const nav2_costmap_2d::Costmap2D& costmap,
        const std::vector<std::pair<double, double>>& map_path) {
      std::vector<std::pair<double, double>> world_path{};
      world_path.reserve(map_path.size());
      for (const auto& [mx, my] : map_path) {
        world_path.emplace_back(mapContinuousToWorld(costmap, mx, my));
      }
      return world_path;
    }

    [[nodiscard]] bool isWorldPointAllowed(
        const nav2_costmap_2d::Costmap2D& costmap, double wx, double wy,
        bool allow_unknown, int cost_threshold) {
      unsigned int mx{};
      unsigned int my{};
      if (!costmap.worldToMap(wx, wy, mx, my)) {
        return false;
      }

      unsigned char cost = costmap.getCost(mx, my);
      if (cost == UNKNOWN_COST) {
        return allow_unknown;
      }
      return cost < cost_threshold;
    }

    [[nodiscard]] bool isWorldSegmentAllowed(
        const nav2_costmap_2d::Costmap2D& costmap, double x0, double y0,
        double x1, double y1, bool allow_unknown, int cost_threshold) {
      double length = std::hypot(x1 - x0, y1 - y0);
      double step = std::max(costmap.getResolution() * 0.5, 1e-3);
      int samples = std::max(1, static_cast<int>(std::ceil(length / step)));

      for (int i = 0; i <= samples; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(samples);
        double wx = x0 + (t * (x1 - x0));
        double wy = y0 + (t * (y1 - y0));
        if (!isWorldPointAllowed(costmap, wx, wy, allow_unknown,
                                 cost_threshold)) {
          return false;
        }
      }
      return true;
    }

    [[nodiscard]] bool isPathCollisionFree(
        const nav_msgs::msg::Path& plan,
        const nav2_costmap_2d::Costmap2D& costmap, bool allow_unknown,
        int cost_threshold) {
      if (plan.poses.empty()) {
        return false;
      }

      for (size_t i = 0; i < plan.poses.size(); ++i) {
        const auto& point = plan.poses[i].pose.position;
        if (!isWorldPointAllowed(costmap, point.x, point.y, allow_unknown,
                                 cost_threshold)) {
          return false;
        }

        if (i == 0) {
          continue;
        }

        const auto& prev = plan.poses[i - 1].pose.position;
        if (!isWorldSegmentAllowed(costmap, prev.x, prev.y, point.x, point.y,
                                   allow_unknown, cost_threshold)) {
          return false;
        }
      }
      return true;
    }

  }  // namespace

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
        node, name_ + ".esdf_safe_distance", rclcpp::ParameterValue(0.6));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".corridor_halfwidth", rclcpp::ParameterValue(8.0));
    // B-spline 平滑路径碰撞判定阈值 (253 严格 / 254 宽松)
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".collision_cost_threshold", rclcpp::ParameterValue(253));
    node->get_parameter(name_ + ".enable_esdf", enable_esdf_);
    node->get_parameter(name_ + ".esdf_weight", esdf_weight_);
    node->get_parameter(name_ + ".esdf_safe_distance", esdf_safe_distance_);
    node->get_parameter(name_ + ".corridor_halfwidth", corridor_halfwidth_);
    bspline_config_.corridor_halfwidth = corridor_halfwidth_;
    node->get_parameter(name_ + ".collision_cost_threshold",
                        collision_cost_threshold_);

    RCLCPP_INFO(
        logger_,
        "JPSPlanner configured: w_traversal=%.2f w_euc=%.2f "
        "w_heuristic=%.2f allow_unknown=%d enable_bspline=%d enable_esdf=%d "
        "esdf_safe_distance=%.2f corridor_halfwidth=%.1f "
        "collision_threshold=%d",
        config_.w_traversal_cost, config_.w_euc_cost, config_.w_heuristic_cost,
        config_.allow_unknown, enable_bspline_, enable_esdf_,
        esdf_safe_distance_, corridor_halfwidth_, collision_cost_threshold_);

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
  //     ▼ mapContinuousToWorld()
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
    unsigned int mx_start{};
    unsigned int my_start{};
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                              mx_start, my_start)) {
      RCLCPP_ERROR(logger_,
                   "JPSPlanner: start pose (%.2f, %.2f) out of costmap bounds",
                   start.pose.position.x, start.pose.position.y);
      return plan;
    }

    unsigned int mx_goal{};
    unsigned int my_goal{};
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

    // 贴障碍的对角段改写为正交移动, 避免 B-spline 平滑切角产生锯齿/回退
    map_path = detourCornerHuggingDiagonals(map_path, state.costmap_data,
                                            state.size_x, state.size_y,
                                            config_.allow_unknown);

    RCLCPP_INFO(logger_, "JPSPlanner: path found with %zu waypoints",
                map_path.size());

    auto make_linear_plan =
        [&](const std::vector<std::pair<double, double>>& linear_map_path) {
          auto world_path = mapPathToWorld(*costmap_, linear_map_path);
          return linearInterpolation(world_path, costmap_->getResolution());
        };

    // ── 第 2 步: B-spline 平滑 + (可选) ESDF 梯度优化 ──
    // 7 阶 B-spline 在稀疏跳点间容易过冲切角 (尤其对角贴障碍段),
    // 所以先按 ≤2 格步长在地图坐标补密, 再进入插值。
    if (enable_bspline_) {
      auto spline_path = densifyMapPath(map_path, BSPLINE_DENSIFY_STEP_CELLS,
                                        MIN_BSPLINE_WAYPOINTS);
      if (spline_path.size() > map_path.size()) {
        RCLCPP_INFO(logger_,
                    "JPSPlanner: densified path from %zu to %zu waypoints for "
                    "B-spline",
                    map_path.size(), spline_path.size());
      }

      if (spline_path.size() >= MIN_BSPLINE_WAYPOINTS) {
        plan = bsplineSmooth(spline_path, costmap_->getCharMap(),
                             static_cast<int>(costmap_->getSizeInCellsX()),
                             static_cast<int>(costmap_->getSizeInCellsY()),
                             costmap_->getResolution());
        RCLCPP_INFO(logger_, "JPSPlanner: B-spline smooth applied, %zu poses",
                    plan.poses.size());
        if (!isPathCollisionFree(plan, *costmap_, config_.allow_unknown,
                                 collision_cost_threshold_)) {
          RCLCPP_WARN(logger_,
                      "JPSPlanner: smoothed path collides with costmap, "
                      "falling back to linear JPS path");
          plan = make_linear_plan(map_path);
        }
      } else {
        RCLCPP_INFO(logger_,
                    "JPSPlanner: too few waypoints (%zu, densified to %zu) for "
                    "B-spline, linear fallback",
                    map_path.size(), spline_path.size());
        plan = make_linear_plan(map_path);
      }
    } else {
      plan = make_linear_plan(map_path);
    }

    if (!isPathCollisionFree(plan, *costmap_, config_.allow_unknown,
                             collision_cost_threshold_)) {
      RCLCPP_ERROR(logger_,
                   "JPSPlanner: final path collides with costmap, "
                   "returning empty plan");
      plan.poses.clear();
    }

    plan.header.stamp = clock_->now();
    plan.header.frame_id = global_frame_;
    for (auto& pose : plan.poses) {
      pose.header = plan.header;
    }

    // 推送路径到共享内存供 UI 渲染
    writePathToShm(plan);

    return plan;
  }

  // ══════════════════════════════════════════════════════════════════════════════
  // bsplineSmooth — B-spline 拟合 + 可选的 ESDF 梯度优化
  //
  // 数学原理:
  //   代价函数 J(P) = w_s·J_smooth + w_d·J_dist + w_e·J_esdf
  //
  //   其中各分量分别为:
  //     J_smooth = ∫‖C''‖² du          — 曲率能量, 保持路径平滑
  //     J_dist   = Σ‖C(τ_i)-q_i‖²     — 偏离原始 JPS 航点的距离
  //     J_esdf   = Σ [max(0, d_safe - d_esdf(p_i))]² — 连续距离场避障
  //
  //   ESDF 代价 J_esdf 仅在启用 enable_esdf 时计算,
  //   此时会自动开启梯度下降 (enable_gradient_descent=true)。
  // ══════════════════════════════════════════════════════════════════════════════

  nav_msgs::msg::Path JPSPlanner::bsplineSmooth(
      const std::vector<std::pair<double, double>>& map_path,
      const unsigned char* costmap_data, int cm_w, int cm_h,
      double resolution) {
    nav_msgs::msg::Path plan;
    bspline_opt::BSplineConfig runtime_config = bspline_config_;
    runtime_config.corridor_halfwidth = corridor_halfwidth_;
    const rog_map_layer::EsdfMap* esdf_map = nullptr;
    double esdf_max_distance = 0.0;

    // ════════════════════════════════════════════════════════════════════════
    // 第 1 步: 查找 ESDF 数据 (启用时)
    //
    // BSplineOptimizer 在构造时复制配置, 所以必须先确定本次规划是否
    // 能拿到 EsdfLayer, 再创建 optimizer。否则当前规划会漏掉 ESDF 代价。
    // ════════════════════════════════════════════════════════════════════════
    if (enable_esdf_) {
      auto* layered_costmap = costmap_ros_->getLayeredCostmap();
      if (layered_costmap != nullptr) {
        auto* plugins = layered_costmap->getPlugins();
        if (plugins != nullptr) {
          for (auto& plugin : *plugins) {
            auto esdf_layer =
                std::dynamic_pointer_cast<rog_map_layer::EsdfLayer>(plugin);
            if (esdf_layer) {
              esdf_map = esdf_layer->getEsdfMapRaw();
              if (esdf_map != nullptr) {
                const auto& esdf_cfg = esdf_layer->config();
                esdf_max_distance = esdf_cfg.max_distance;
                runtime_config.enable_esdf = true;
                runtime_config.enable_gradient_descent = true;
                runtime_config.esdf_weight = esdf_weight_;
                runtime_config.esdf_safe_distance = esdf_safe_distance_;

                RCLCPP_INFO(
                    logger_,
                    "JPSPlanner: ESDF layer found, enabling gradient descent "
                    "with w_esdf=%.1f safe_dist=%.2f corridor=%.1f cells",
                    esdf_weight_, esdf_safe_distance_,
                    runtime_config.corridor_halfwidth);
              }
              break;
            }
          }
        }
      }
      if (!runtime_config.enable_esdf) {
        RCLCPP_WARN(logger_,
                    "JPSPlanner: ESDF enabled but EsdfLayer not found in "
                    "costmap plugins, "
                    "falling back to control-point spiral projection.");
      }
    }

    // ── 第 2 步: 创建 B-spline 优化器并拟合 JPS 路径 ──
    // fit() 使用 chord-length 参数化 + Eigen SplineFitting::Interpolate,
    // 将 JPS 航点精确插值为 7 阶 C2 连续 B-spline 曲线
    bspline_opt::BSplineOptimizer opt(runtime_config);
    if (!opt.fit(map_path)) {
      RCLCPP_WARN(
          logger_,
          "JPSPlanner: B-spline fit failed, using linear interpolation");
      auto world_path = mapPathToWorld(*costmap_, map_path);
      return linearInterpolation(world_path, resolution);
    }

    // 注入代价地图 — 用于内部控制点的障碍物避让 (cost ≥ 253 = 障碍物)
    opt.state().costmap_data = costmap_data;
    opt.state().costmap_w = cm_w;
    opt.state().costmap_h = cm_h;

    // ════════════════════════════════════════════════════════════════════════
    // 第 3 步: 注入 ESDF 数据 (启用时)
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
    if (esdf_map != nullptr) {
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
      opt.state().esdf_max_distance = esdf_max_distance;
    }

    // ── 第 4 步: 运行优化 ──
    // 变量 = 内部控制点 [2*(M-2) 维]
    // 方法 = 解析梯度 (平滑/距离/ESDF) + 回溯线搜索
    // 约束 = |x_i - x_init| ≤ corridor_halfwidth (走廊约束)
    // 避障 = (a) ESDF 梯度平滑推离 (启用时)
    //        (b) 控制点螺旋搜索投射到最近空闲格元
    // 输出 = smoothed_path + curvature_profile
    // 输出航点数量上限: 长路径下防止采样点过多拖慢后处理和下游 controller。
    // 1000 个点按 0.05m 分辨率覆盖 50m 路径, 足够稠密。
    int num_samples = std::min(
        1000, std::max(100, static_cast<int>(map_path.size()) * 5));
    auto result = opt.optimize(num_samples);

    // ── 第 5 步: 地图坐标 → 世界坐标 ──
    plan.poses.reserve(result.smoothed_path.size());
    for (size_t i = 0; i < result.smoothed_path.size(); ++i) {
      const auto [wx, wy] = mapContinuousToWorld(
          *costmap_, result.smoothed_path[i].first,
          result.smoothed_path[i].second);
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
        pose.pose.position.x = x0 + (t * (x1 - x0));
        pose.pose.position.y = y0 + (t * (y1 - y0));
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
                        : ((n / guga_ui::UI_PATH_MAX_POINTS) + 1);
    ui_path.count = std::min((n + stride - 1) / stride,
                             guga_ui::UI_PATH_MAX_POINTS);

    for (uint32_t i = 0; i < ui_path.count; ++i) {
      size_t src = i * stride;
      ui_path.x[i] = plan.poses[src].pose.position.x;
      ui_path.y[i] = plan.poses[src].pose.position.y;
    }

    shm_writer_.write(&ui_path, sizeof(ui_path));
  }

}  // namespace jps_planner

PLUGINLIB_EXPORT_CLASS(jps_planner::JPSPlanner, nav2_core::GlobalPlanner)
