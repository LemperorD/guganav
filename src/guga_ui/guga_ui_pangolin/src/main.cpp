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

#include <csignal>
#include <iostream>
#include <memory>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/default_font.h>
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

static void sigintHandler(int /*signum*/) { g_running = 0; }

// ==================== 主函数 ====================

int main(int argc, char* argv[]) {
  
  return 0;
}
