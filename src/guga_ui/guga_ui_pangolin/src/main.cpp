/**
 * @file main.cpp
 * @brief guga_ui_pangolin — RoboMaster 哨兵 UI 主程序入口。
 *
 * 架构：
 *   main() → 打开 shm → 初始化 Pangolin 窗口和布局 → 每帧调用
 *   DataSource::update() 刷新数据 → 各 Render 函数绘制 → FinishFrame()。
 *
 * 键盘快捷键：
 *   - 1/2/3: 切换底盘模式（预留控制通道）
 *   - V: 切换 3D 视图（顶视 ↔ 跟随）
 *   - R: 重置视角
 *   - ESC: 退出
 */

#include <clocale>
#include <csignal>
#include <iostream>
#include <locale>
#include <memory>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/default_font.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/opengl_render_state.h>

#include "guga_ui_common/shm_reader.hpp"

// 子模块声明
#include "data_source.hpp"
#include "render_3d.hpp"
#include "render_decision.hpp"
#include "render_hud.hpp"

// ==================== 全局标志 ====================

/// 收到 SIGINT 时置位
static volatile sig_atomic_t g_running{1};

static void sigintHandler(int /*signum*/) { g_running = 0; }

// ==================== 主函数 ====================

int main(int argc, char* argv[]) {
  std::setlocale(LC_ALL, "");
  std::locale::global(std::locale(""));

  // 注册信号处理
  signal(SIGINT, sigintHandler);

  const char* shm_name = guga_ui::SHM_DEFAULT_NAME;
  if (argc > 1) {
    shm_name = argv[1];
  }

  // ---- 1. 打开共享内存 ----
  UiDataSource data_source;
  if (!data_source.open(shm_name)) {
    std::cerr << "[guga_ui] Failed to open shared memory \"" << shm_name
              << "\". Is any writer module running?" << std::endl;
    // 不退出，允许无数据时显示空白窗口
  }

  // ---- 2. 创建 Pangolin 窗口 ----
  pangolin::CreateWindowAndBind("GUGA Sentry UI", 1280, 800);

  // ---- 3. 布局 ----
  // 将窗口分为左侧 3D 视图 + 右侧面板（HUD 在上，决策在下）
  pangolin::View& root_view =
      pangolin::DisplayBase()
          .SetBounds(0.0, 1.0, 0.0, 1.0)
          .SetLayout(pangolin::LayoutEqualHorizontal);

  // 3D 视图 — 左侧
  pangolin::View& view_3d =
      root_view.AddDisplay(pangolin::View()
                               .SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(800)));

  // 右侧面板 — 垂直分割
  pangolin::View& right_panel =
      root_view.AddDisplay(pangolin::View()
                               .SetBounds(0.0, 1.0,
                                          pangolin::Attach::Pix(800), 1.0)
                               .SetLayout(pangolin::LayoutEqualVertical));

  // HUD 面板 — 右上方
  pangolin::View& hud_view =
      right_panel.AddDisplay(pangolin::View().SetBounds(
          0.0, 0.5, 0.0, 1.0));

  // 决策面板 — 右下方
  pangolin::View& decision_view =
      right_panel.AddDisplay(pangolin::View().SetBounds(
          0.5, 1.0, 0.0, 1.0));

  // ---- 4. 3D 相机状态 ----
  GugaRender3D render_3d;
  render_3d.init(view_3d);

  GugaRenderHUD render_hud;
  GugaRenderDecision render_decision;

  // ---- 5. 设置自定义绘制回调 ----
  view_3d.SetDrawFunction([&](pangolin::View& v) {
    render_3d.draw(v, data_source);
  });

  hud_view.SetDrawFunction([&](pangolin::View& v) {
    render_hud.draw(v, data_source);
  });

  decision_view.SetDrawFunction([&](pangolin::View& v) {
    render_decision.draw(v, data_source);
  });

  // ---- 6. 主循环 ----
  std::cout << "[guga_ui] Entering render loop..." << std::endl;

  while (g_running && !pangolin::ShouldQuit()) {
    // 每帧刷新共享内存数据
    data_source.update();
    pangolin::FinishFrame();
  }

  pangolin::GetBoundWindow()->RemoveCurrent();
  std::cout << "[guga_ui] Shutting down." << std::endl;
  return 0;
}
