#ifndef GUGA_UI_PANGOLIN_PANGOLIN_WINDOW_HPP
#define GUGA_UI_PANGOLIN_PANGOLIN_WINDOW_HPP

#include <atomic>
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
  explicit PangolinWindow();

  /**
   * @brief 析构函数
   */
  ~PangolinWindow() = default;

  /**
   * @brief 渲染函数
   */
  void Render();

  /**
   * @brief 获取窗口名称
   * @return 窗口名称
   */
  std::string GetWindowName() const;

public:
  std::thread render_thread_; // 渲染线程
  std::atomic<bool> exit_flag_;

private: // 方法
  void CreateDisplayLayout();

  /// 创建OpenGL Buffers
  void AllocateBuffer();
  void ReleaseBuffer();

private: // 成员变量
  /// 窗口layout相关
  const std::string window_title_ = "GUGA_UI";
  int win_width_ = 1920;
  int win_height_ = 1080;
  static constexpr float cam_focus_ = 5000;
  static constexpr float cam_z_near_ = 1.0;
  static constexpr float cam_z_far_ = 1e10;
  static constexpr int menu_width_ = 210;

  /// text
  pangolin::GlText gltext_label_global_;

  /// camera
  pangolin::OpenGlRenderState s_cam_main_;

  UiDataSource ui_data_source_;
};

} // namespace guga_ui

#endif // GUGA_UI_PANGOLIN_PANGOLIN_WINDOW_HPP