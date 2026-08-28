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

  inline void CreatePanelDecision(const UiDataSource& ui_data_source) {
    // 左下：决策面板（下半个面板列）
    pangolin::CreatePanel("menu").SetBounds(
        0.0, 0.5, 0.0, pangolin::Attach::Pix(UiPanelWidth()));
  }

}  // namespace guga_ui

#endif  // PANEL_DECISION_HPP
