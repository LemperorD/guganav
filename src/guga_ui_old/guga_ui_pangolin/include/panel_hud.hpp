#ifndef GUGA_UI_PANGOLIN_PANEL_HUD_HPP
#define GUGA_UI_PANGOLIN_PANEL_HUD_HPP

#include <cstdint>
#include <cstdio>
#include <string>

#include "panel_const.hpp"

namespace guga_ui
{

void CreatePanelHud(pangolin::View& view, const UiDataSource& ui_data_source) {
  pangolin::CreatePanel("HUD").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

}

}  // namespace guga_ui

#endif // GUGA_UI_PANGOLIN_PANEL_HUD_HPP