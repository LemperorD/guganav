#ifndef GUGA_UI_PANGOLIN_PANEL_HUD_HPP
#define GUGA_UI_PANGOLIN_PANEL_HUD_HPP

#include <cstdint>
#include <cstdio>
#include <string>

#include "panel_const.hpp"

namespace guga_ui
{

inline void CreatePanelHud(const UiDataSource& ui_data_source) {
  // 左上：HUD 面板（上半个面板列）
  pangolin::CreatePanel("HUD").SetBounds(0.5, 1.0, 0.0, pangolin::Attach::Pix(UiPanelWidth()));

}

}  // namespace guga_ui

#endif // GUGA_UI_PANGOLIN_PANEL_HUD_HPP
