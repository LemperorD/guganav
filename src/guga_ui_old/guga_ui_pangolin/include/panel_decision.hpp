#ifndef GUGA_UI_PANGOLIN_PANEL_DECISION_HPP
#define GUGA_UI_PANGOLIN_PANEL_DECISION_HPP
/**
 * @file panel_decision.hpp
 * @brief Decision panel renderer with panel aesthetic.
 *
 * Draws a bordered panel with title bar in pixel coordinates,
 * showing decision state, chassis mode, robot pose, nav goal,
 * gimbal yaw, and compute time.
 */

#include <cstdint>
#include <cstdio>
#include <string>
#include "data_source.hpp"
#include "panel_const.hpp"
#include <pangolin/pangolin.h>
namespace guga_ui {
  class PanelDecision {
  public:
    void init(const UiDataSource& ui_data_source) {
      // 左下：决策面板（下半个面板列）
      pangolin::CreatePanel("menu").SetBounds(
          0.0, 0.5, 0.0, pangolin::Attach::Pix(UiPanelWidth()));
    }
    void update(const UiDataSource& ui_data_source) {
      const auto& d = ui_data_source.data();
      // 决策槽位从未写入时显示 "--"，避免把 0 误当成真实目标点。
      if (ui_data_source.isFresh(d.game_status_age, 60)) {
        match_status_ = std::to_string(d.game_status.game_progress);
      } else {
        match_status_ = "--";
      }
    }

  private:
    pangolin::Var<std::string> match_status_{"menu.Match Status", "0"};
  };
}  // namespace guga_ui

#endif  // PANEL_DECISION_HPP
