#ifndef PANGOLIN_WINDOW_HPP
#define PANGOLIN_WINDOW_HPP

#include <csignal>
#include <iostream>
#include <memory>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/default_font.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/opengl_render_state.h>

#include "render_3d.hpp"
#include "panel_decision.hpp"
#include "panel_hud.hpp"

namespace guga_ui
{

class PangolinWindow
{
public: // 构造和析构
  /**
   * @brief 构造函数
   * @param window_title 窗口标题
   * @param width 窗口宽度
   * @param height 窗口高度
   */
  explicit PangolinWindow(const std::string& window_title = "GUGA UI", const int width = 1920, const int height = 1080);

  /**
   * @brief 析构函数
   */
  ~PangolinWindow() = default;

private:
  
};

} // namespace guga_ui

#endif // PANGOLIN_WINDOW_HPP