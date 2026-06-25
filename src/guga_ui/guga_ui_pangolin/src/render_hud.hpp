#ifndef GUGA_UI_PANGOLIN_RENDER_HUD_HPP
#define GUGA_UI_PANGOLIN_RENDER_HUD_HPP
/**
 * @file render_hud.hpp
 * @brief HUD 状态面板渲染：HP、热量、弹量、金币、比赛阶段、倒计时、RFID 增益点。
 *
 * 使用 Pangolin 内置的 glDraw 函数在像素坐标系中绘制文字和图形。
 * 所有绘制在 View 的 ActivatePixelOrthographic() 激活的 2D 坐标系中进行。
 */

#include <cstdint>
#include <cstdio>
#include <string>

#include <pangolin/display/default_font.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include "data_source.hpp"

class GugaRenderHUD {
public:
  void draw(pangolin::View& view, const UiDataSource& ds) {
    // 激活 2D 像素坐标（x 右，y 下）
    view.ActivatePixelOrthographic();
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    const auto& d = ds.data();
    double y{20.0};
    const double line_h{22.0};
    const double margin{15.0};

    // ---- 标题 ----
    drawText(margin, y, "[GUGA Sentry UI]");
    y += line_h * 1.2;

    if (!d.has_data) {
      glColor4f(1.0f, 0.3f, 0.3f, 1.0f);
      drawText(margin, y, "NO DATA — Waiting for writer...");
      return;
    }

    // ---- 比赛状态 ----
    drawText(margin, y, "=== GAME ===");
    y += line_h;

    if (ds.isFresh(d.game_status_age, 120)) {
      char buf[64];
      const char* phase_str = gameProgressStr(d.game_status.game_progress);
      std::snprintf(buf, sizeof(buf), "Phase: %s | Remaining: %ds",
                    phase_str, d.game_status.stage_remain_time);
      drawText(margin, y, buf);
      y += line_h;

      std::snprintf(buf, sizeof(buf), "Elapsed: %.1fs",
                    d.game_status.elapsed_sec);
      drawText(margin, y, buf);
    } else {
      drawTextDim(margin, y, "Game: no data");
    }
    y += line_h * 1.5;

    // ---- 机器人状态 ----
    drawText(margin, y, "=== ROBOT ===");
    y += line_h;

    if (ds.isFresh(d.robot_status_age, 60)) {
      const auto& rs = d.robot_status;

      // HP 条
      char buf[128];

      float hp_ratio = (rs.maximum_hp > 0)
                         ? static_cast<float>(rs.current_hp)
                             / static_cast<float>(rs.maximum_hp)
                         : 0.0f;
      // 根据血量百分比切换颜色
      if (hp_ratio > 0.5f) {
        glColor4f(0.2f, 0.9f, 0.3f, 1.0f);  // 绿
      } else if (hp_ratio > 0.2f) {
        glColor4f(1.0f, 0.8f, 0.0f, 1.0f);  // 黄
      } else {
        glColor4f(1.0f, 0.2f, 0.2f, 1.0f);  // 红
      }
      drawBar(margin, y, 300.0, 16.0, hp_ratio);

      std::snprintf(buf, sizeof(buf), "  HP: %u / %u", rs.current_hp,
                    rs.maximum_hp);
      drawText(margin + 310.0, y + 14.0, buf);
      y += line_h + 4.0;

      // 热量条
      float heat_ratio = (rs.shooter_barrel_heat_limit > 0)
                           ? static_cast<float>(
                               rs.shooter_17mm_1_barrel_heat)
                               / static_cast<float>(
                                   rs.shooter_barrel_heat_limit)
                           : 0.0f;
      glColor4f(1.0f, 0.5f, 0.0f, 1.0f);
      drawBar(margin, y, 300.0, 16.0, heat_ratio);

      std::snprintf(buf, sizeof(buf), "  Heat: %u / %u",
                    rs.shooter_17mm_1_barrel_heat,
                    rs.shooter_barrel_heat_limit);
      drawText(margin + 310.0, y + 14.0, buf);
      y += line_h + 4.0;

      // 弹量和金币
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      std::snprintf(buf, sizeof(buf), "Ammo17: %u   Gold: %u",
                    rs.projectile_allowance_17mm,
                    rs.remaining_gold_coin);
      drawText(margin, y + 5.0, buf);
      y += line_h;

      // 受击提示
      if (rs.is_hp_deduced) {
        glColor4f(1.0f, 0.2f, 0.2f, 1.0f);
        drawText(margin, y + 5.0, ">> UNDER ATTACK <<");
      }
    } else {
      drawTextDim(margin, y, "Robot: no data");
    }
    y += line_h * 1.5;

    // ---- RFID 增益点 ----
    drawText(margin, y, "=== RFID ===");
    y += line_h;

    if (d.rfid_status.base_gain_point) {
      drawRfidTag(margin, y, "BASE");
      y += line_h;
    }
    if (d.rfid_status.central_highland_gain_point) {
      drawRfidTag(margin, y, "CENTER_HIGHLAND");
      y += line_h;
    }
    if (d.rfid_status.friendly_outpost_gain_point) {
      drawRfidTag(margin, y, "OUTPOST");
      y += line_h;
    }
    if (d.rfid_status.friendly_supply_zone_non_exchange
        || d.rfid_status.friendly_supply_zone_exchange) {
      drawRfidTag(margin, y, "SUPPLY");
      y += line_h;
    }
    if (d.rfid_status.friendly_big_resource_island) {
      drawRfidTag(margin, y, "RESOURCE_ISLAND");
      y += line_h;
    }
  }

private:
  // ---- 帮助函数 ----

  void drawText(double x, double y, const std::string& text) {
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    pangolin::default_font().Text("%s", text.c_str()).Draw(x, y);
  }

  void drawTextDim(double x, double y, const std::string& text) {
    glColor4f(0.4f, 0.4f, 0.5f, 1.0f);
    pangolin::default_font().Text("%s", text.c_str()).Draw(x, y);
  }

  /**
   * @brief 绘制水平进度条。
   * @param x, y 左上角坐标。
   * @param w, h 条宽高。
   * @param ratio 0.0–1.0 的填充比例。
   */
  void drawBar(double x, double y, double w, double h, float ratio) {
    // Save current color so the fill bar keeps the caller's color
    GLfloat saved[4];
    glGetFloatv(GL_CURRENT_COLOR, saved);

    // Background
    glColor4f(0.2f, 0.2f, 0.2f, 0.8f);
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();

    // Fill bar (uses color saved before drawBar was called)
    glColor4f(saved[0], saved[1], saved[2], saved[3]);
    const float fill_w = w * ratio;
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + fill_w, y);
    glVertex2f(x + fill_w, y + h);
    glVertex2f(x, y + h);
    glEnd();
  }

  void drawRfidTag(double x, double y, const std::string& label) {
    glColor4f(0.3f, 0.8f, 0.3f, 1.0f);
    pangolin::default_font().Text("  [*] %s", label.c_str()).Draw(x, y);
  }

  const char* gameProgressStr(uint8_t progress) {
    switch (progress) {
      case 0: return "NOT_START";
      case 1: return "PREPARE";
      case 2: return "SELF_CHECK";
      case 3: return "COUNTDOWN";
      case 4: return "RUNNING";
      case 5: return "GAME_OVER";
      default: return "UNKNOWN";
    }
  }
};

#endif  // GUGA_UI_PANGOLIN_RENDER_HUD_HPP
