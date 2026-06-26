#ifndef GUGA_UI_PANGOLIN_PANEL_HUD_HPP
#define GUGA_UI_PANGOLIN_PANEL_HUD_HPP

#include <cstdint>
#include <cstdio>
#include <string>

#include <pangolin/display/default_font.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include "data_source.hpp"

namespace guga_ui
{

class PanelHUD
{
public: // 构造和析构
  explicit PanelHUD();
  ~PanelHUD() = default;

private:

};

}  // namespace guga_ui

#endif // GUGA_UI_PANGOLIN_PANEL_HUD_HPP