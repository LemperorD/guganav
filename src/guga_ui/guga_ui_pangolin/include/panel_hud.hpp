#ifndef GUGA_UI_PANGOLIN_PANEL_HUD_HPP
#define GUGA_UI_PANGOLIN_PANEL_HUD_HPP

#include <cstdint>
#include <cstdio>
#include <string>

#include <pangolin/display/default_font.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

const int UI_WIDTH = 20* pangolin::default_font().MaxWidth();

namespace guga_ui
{

void CreatePanelHud(pangolin::View& view, const UiDataSource& ui_data_source) {
  pangolin::CreatePanel("HUD").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

}

}  // namespace guga_ui

#endif // GUGA_UI_PANGOLIN_PANEL_HUD_HPP