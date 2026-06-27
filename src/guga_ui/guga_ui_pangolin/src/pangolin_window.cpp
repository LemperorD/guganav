#include "guga_ui_pangolin/pangolin_window.hpp"

namespace guga_ui
{

PangolinWindow::PangolinWindow()
{
  // 创建 Pangolin 窗口
  pangolin::CreateWindowAndBind(window_title_, win_width_, win_height_);

  CreateDisplayLayout();
}

void PangolinWindow::CreateDisplayLayout() {
  // define camera render object (for view / scene browsing)
  // 定义视点的透视投影方式
  auto proj_mat_main = pangolin::ProjectionMatrix(win_width_, win_width_, cam_focus_, cam_focus_, win_width_ / 2,
                                                  win_width_ / 2, cam_z_near_, cam_z_far_);
  // 模型视图矩阵定义了视点的位置和朝向
  auto model_view_main = pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, pangolin::AxisY);
  s_cam_main_ = pangolin::OpenGlRenderState(std::move(proj_mat_main), std::move(model_view_main));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam3d_main = pangolin::Display(dis_3d_main_name_)
                                      .SetBounds(0.0, 1.0, 0.0, 1.0)
                                      .SetHandler(new pangolin::Handler3D(s_cam_main_));

  pangolin::View &d_cam3d = pangolin::Display(dis_3d_name_)
                                .SetBounds(0.0, 1.0, 0.0, 0.75)
                                .SetLayout(pangolin::LayoutOverlay)
                                .AddDisplay(d_cam3d_main);

  // OpenGL 'view' of data. We might have many views of the same data.
  plotter_vel_ = std::make_unique<pangolin::Plotter>(&log_vel_, -10, 600, -11, 11, 75, 2);
  plotter_vel_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_vel_->Track("$i");
  plotter_vel_baselink_ = std::make_unique<pangolin::Plotter>(&log_vel_baselink_, -10, 600, -11, 11, 75, 2);
  plotter_vel_baselink_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_vel_baselink_->Track("$i");
  plotter_confidence_ = std::make_unique<pangolin::Plotter>(&log_confidence_, -10, 600, 0, 5.0, 100, 0.5);
  plotter_confidence_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_confidence_->Track("$i");
  plotter_err_ = std::make_unique<pangolin::Plotter>(&log_error_, -10, 600, 0, 1.0, 100, 0.1);
  plotter_err_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_err_->Track("$i");

  pangolin::View &d_plot = pangolin::Display(dis_plot_name_)
                                .SetBounds(0.0, 1.0, 0.75, 1.0)
                                .SetLayout(pangolin::LayoutEqualVertical)
                                .AddDisplay(*plotter_confidence_)
                                .AddDisplay(*plotter_err_)
                                .AddDisplay(*plotter_vel_)
                                .AddDisplay(*plotter_vel_baselink_);
  pangolin::Display(dis_main_name_)
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(menu_width_), 1.0)
      .AddDisplay(d_cam3d)
      .AddDisplay(d_plot);
}

void PangolinWindow::Render() {
	pangolin::BindToContext(window_title_);

	// Issue specific OpenGl we might need
	// 启用OpenGL深度测试和混合功能，以支持透明度等效果。
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// display layout
	CreateDisplayLayout();

	exit_flag_.store(false);
	while (!pangolin::ShouldQuit() && !exit_flag_) {
		// Clear entire screen
		glClearColor(20.0 / 255.0, 20.0 / 255.0, 20.0 / 255.0, 1.0);
		// 清除了颜色缓冲区（GL_COLOR_BUFFER_BIT）和深度缓冲区（GL_DEPTH_BUFFER_BIT）。
		// 通常在每一帧渲染之前执行的操作，以准备渲染新的内容。
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		pangolin::FinishFrame();
		std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200Hz
	}

	// unset the current context from the main thread
	pangolin::GetBoundWindow()->RemoveCurrent();
	pangolin::DestroyWindow(GetWindowName());
}

std::string PangolinWindow::GetWindowName() const { return win_name_; }

void PangolinWindow::AllocateBuffer() {
  std::string global_text("Welcome to GUGA_UI\n");
  auto &font = pangolin::default_font();
  gltext_label_global_ = font.Text(global_text);
}

void PangolinWindow::ReleaseBuffer() {

}

} // namespace guga_ui