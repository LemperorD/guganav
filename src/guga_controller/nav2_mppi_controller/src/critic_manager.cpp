// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey
// Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_controller/critic_manager.hpp"

#include <chrono>
#include <cstdio>
#include <string>

namespace mppi {

  void CriticManager::on_configure(
      rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string& name,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
      ParametersHandler* param_handler) {
    parent_ = parent;
    costmap_ros_ = costmap_ros;
    name_ = name;
    auto node = parent_.lock();
    logger_ = node->get_logger();
    parameters_handler_ = param_handler;

    getParams();
    loadCritics();
  }

  void CriticManager::getParams() {
    auto node = parent_.lock();
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(critic_names_, "critics", std::vector<std::string>{},
             ParameterType::Static);
    // 打开后每 2 s 打一行各 critic 的单周期耗时，用于定位"算不过来"的时间去哪了
    getParam(debug_timing_, "debug_timing", false);
  }

  void CriticManager::loadCritics() {
    if (!loader_) {
      loader_ =
          std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
              "nav2_mppi_controller", "mppi::critics::CriticFunction");
    }

    critics_.clear();
    for (auto name : critic_names_) {
      std::string fullname = getFullName(name);
      auto instance = std::unique_ptr<critics::CriticFunction>(
          loader_->createUnmanagedInstance(fullname));
      critics_.push_back(std::move(instance));
      critics_.back()->on_configure(parent_, name_, name_ + "." + name,
                                    costmap_ros_, parameters_handler_);
      RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
    }
  }

  std::string CriticManager::getFullName(const std::string& name) {
    return "mppi::critics::" + name;
  }

  void CriticManager::evalTrajectoriesScores(CriticData& data) const {
    if (debug_timing_ && critic_ms_.size() != critics_.size()) {
      critic_ms_.assign(critics_.size(), 0.0);
    }
    for (size_t q = 0; q < critics_.size(); q++) {
      if (data.fail_flag) {
        break;
      }
      if (!debug_timing_) {
        critics_[q]->score(data);
        continue;
      }
      const auto t0 = std::chrono::steady_clock::now();
      critics_[q]->score(data);
      critic_ms_[q] = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    }

    if (debug_timing_) {
      std::string report;
      double total = 0.0;
      char buf[80];
      for (size_t q = 0; q < critics_.size(); q++) {
        total += critic_ms_[q];
        snprintf(buf, sizeof(buf), " %s=%.1f", critic_names_[q].c_str(), critic_ms_[q]);
        report += buf;
      }
      auto node = parent_.lock();
      if (node) {
        RCLCPP_INFO_THROTTLE(
          logger_, *node->get_clock(), 2000, "MPPI critics 合计 %.1f ms:%s", total,
          report.c_str());
      }
    }
  }

}  // namespace mppi
