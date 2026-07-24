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

#include "panel_const.hpp"

namespace guga_ui
{

void CreatePanelDecision(pangolin::View& view, const UiDataSource& ui_data_source) {
  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
  
}

}  // namespace guga_ui

#endif // PANEL_DECISION_HPP