#include "guga_ui_pangolin/pangolin_window.hpp"

namespace guga_ui
{

PangolinWindow::PangolinWindow(const std::string& window_title, const int width, const int height)
  : window_title_(window_title), width_(width), height_(height)
{
  // 创建 Pangolin 窗口
  pangolin::CreateWindowAndBind(window_title_, width_, height_);
}

} // namespace guga_ui