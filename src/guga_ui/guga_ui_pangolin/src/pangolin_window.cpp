#include "guga_ui_pangolin/pangolin_window.hpp"

namespace guga_ui
{

PangolinWindow::PangolinWindow(const std::string& window_title, const int width, const int height)
  : window_title_(window_title), width_(width), height_(height)
{
  // 创建 Pangolin 窗口
  pangolin::CreateWindowAndBind(window_title_, width_, height_);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1920,1080,420,420,960,540,0.1,1000),
    pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
  );

  // Choose a sensible left UI Panel width based on the width of 20
  // charectors from the default font.
  const int UI_WIDTH = 20* pangolin::default_font().MaxWidth();
  // std::cout << "UI_WIDTH: " << UI_WIDTH << std::endl;

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 1920.0f/1080.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

}

} // namespace guga_ui