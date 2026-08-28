/**
 * @file main.cpp
 * @brief guga_ui_pangolin — RoboMaster 哨兵 UI 主程序入口。
 *
 * 架构：
 *   main() → 打开 shm → 初始化 Pangolin 窗口和布局 → 每帧调用
 *   DataSource::update() 刷新数据 → 各 Render 函数绘制 → FinishFrame()。
 *
 * 键盘快捷键：
 *   - V: 切换 3D 视图（顶视 ↔ 跟随，预留）
 *   - ESC: 退出
 */

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/opengl_render_state.h>

// 子模块声明
#include "data_source.hpp"
#include "render_3d.hpp"
#include "panel_decision.hpp"
#include "panel_hud.hpp"

// ==================== 全局标志 ====================

/// 收到 SIGINT 时置位
static volatile sig_atomic_t g_running{1};

static void sigintHandler(int /*signum*/) {
  g_running = 0;
}

// ==================== 主函数 ====================

int main(int argc, char* argv[]) {
  // 注册信号处理：Ctrl+C 退出渲染循环
  signal(SIGINT, sigintHandler);

  // 共享内存名称，默认 "guga_shm"，可用命令行第一个参数覆盖
  const char* shm_name = guga_ui::SHM_DEFAULT_NAME;
  if (argc > 1) {
    shm_name = argv[1];
  }

  // ---- 1. 打开共享内存 ----
  UiDataSource data_source;
  if (!data_source.open(shm_name)) {
    std::cerr << "[guga_ui] 共享内存 \"" << shm_name
              << "\" 打开失败，将以空数据启动（请确认写入端已运行）"
              << std::endl;
  }

  // ---- 2. 创建 Pangolin 窗口 ----
  // 注意：必须在任何 pangolin::default_font() 调用之前，否则会 abort。
  pangolin::CreateWindowAndBind("GUGA Sentry UI", 1920, 1200);

  // ---- 3. 布局 ----
  // 左侧为面板列（HUD 上 / 决策下），右侧为 3D 视图
  const int panel_width = UiPanelWidth();
  guga_ui::PanelHud hud_panel;
  hud_panel.init(data_source);
  guga_ui::CreatePanelDecision(data_source);

  pangolin::View& view_3d = pangolin::Display("view_3d").SetBounds(
      0.0, 1.0, pangolin::Attach::Pix(panel_width), 1.0);

  GugaRender3D render_3d;
  render_3d.init(view_3d);
  view_3d.SetDrawFunction(
      [&](pangolin::View& v) { render_3d.draw(v, data_source); });

  // ---- 4. 主循环 ----
  std::cout << "[guga_ui] 进入渲染循环（ESC / Ctrl+C 退出）..." << std::endl;

  while (g_running && !pangolin::ShouldQuit()) {
    data_source.update();           // 每帧刷新共享内存数据
    hud_panel.update(data_source);  // 刷新 HUD 比赛时间
    pangolin::FinishFrame();        // 渲染所有视图并交换缓冲
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cout << "[guga_ui] 退出。" << std::endl;
  return 0;
}
