#ifndef PANEL_HUD_HPP
#define PANEL_HUD_HPP
/**
 * @file panel_hud.hpp
 * @brief HUD status panel renderer with panel aesthetic.
 *
 * Draws a bordered panel with title bar in pixel coordinates,
 * showing game phase, HP/heat/ammo/gold, under-attack warning,
 * and active RFID gain points.
 */

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
public:
  void draw(pangolin::View& view, const UiDataSource& ds) {
    view.ActivatePixelOrthographic();
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    const auto& d = ds.data();

    const auto vp = view.v;
    const double panel_w = static_cast<double>(vp.w);
    const double panel_h = static_cast<double>(vp.h);
    const double pad{6.0};

    // Panel background
    drawPanelBackground(pad, pad, panel_w - pad * 2, panel_h - pad * 2);

    // Title bar
    const double title_h{28.0};
    drawTitleBar(pad, pad, panel_w - pad * 2, title_h, "HUD STATUS",
                 {0.2f, 0.6f, 0.9f, 1.0f});

    // Content area
    double y{pad + title_h + 8.0};
    const double line_h{20.0};
    const double margin{pad + 10.0};
    const double content_w{panel_w - pad * 2 - 20.0};

    if (!d.has_data) {
      glColor4f(1.0f, 0.3f, 0.3f, 1.0f);
      drawText(margin, y, "NO DATA - Waiting for writer...");
      return;
    }

    // ========== GAME ==========
    drawSectionHeader(margin, y, content_w, "[ GAME ]",
                      {0.3f, 0.55f, 0.8f, 1.0f});
    y += line_h + 2.0;

    if (ds.isFresh(d.game_status_age, 120)) {
      char buf[64];
      const char* phase_str = gameProgressStr(d.game_status.game_progress);
      std::snprintf(buf, sizeof(buf), "Phase: %s", phase_str);
      drawText(margin, y, buf);
      y += line_h;

      std::snprintf(buf, sizeof(buf), "Remaining: %ds",
                    d.game_status.stage_remain_time);
      drawText(margin, y, buf);
      y += line_h;

      std::snprintf(buf, sizeof(buf), "Elapsed: %.1fs",
                    d.game_status.elapsed_sec);
      drawTextDim(margin, y, buf);
    } else {
      drawTextDim(margin, y, "Game: no data");
    }
    y += line_h + 4.0;

    // ========== ROBOT ==========
    drawSectionHeader(margin, y, content_w, "[ ROBOT ]",
                      {0.3f, 0.55f, 0.8f, 1.0f});
    y += line_h + 2.0;

    if (ds.isFresh(d.robot_status_age, 60)) {
      const auto& rs = d.robot_status;
      char buf[128];

      // HP bar
      float hp_ratio = (rs.maximum_hp > 0)
                           ? static_cast<float>(rs.current_hp)
                               / static_cast<float>(rs.maximum_hp)
                           : 0.0f;

      drawLabeledBar(margin, y, content_w, 14.0, hp_ratio, hpColor(hp_ratio),
                     "HP");
      std::snprintf(buf, sizeof(buf), "%u / %u", rs.current_hp, rs.maximum_hp);
      drawTextRight(margin + content_w, y + 12.0, buf, hpColor(hp_ratio));
      y += line_h + 2.0;

      // Heat bar
      float heat_ratio =
          (rs.shooter_barrel_heat_limit > 0)
              ? static_cast<float>(rs.shooter_17mm_1_barrel_heat)
                  / static_cast<float>(rs.shooter_barrel_heat_limit)
              : 0.0f;

      drawLabeledBar(margin, y, content_w, 14.0, heat_ratio,
                     heatColor(heat_ratio), "HEAT");
      std::snprintf(buf, sizeof(buf), "%u / %u",
                    rs.shooter_17mm_1_barrel_heat,
                    rs.shooter_barrel_heat_limit);
      drawTextRight(margin + content_w, y + 12.0, buf, heatColor(heat_ratio));
      y += line_h + 2.0;

      // Ammo
      drawLabel(margin, y, "AMMO 17mm");
      std::snprintf(buf, sizeof(buf), "%u", rs.projectile_allowance_17mm);
      drawTextRight(margin + content_w, y, buf);
      y += line_h;

      // Gold
      drawLabel(margin, y, "GOLD");
      std::snprintf(buf, sizeof(buf), "%u", rs.remaining_gold_coin);
      drawTextRight(margin + content_w, y, buf, {1.0f, 0.85f, 0.2f, 1.0f});
      y += line_h;

      // Under attack warning
      if (rs.is_hp_deduced) {
        y += 4.0;
        glColor4f(1.0f, 0.15f, 0.15f, 1.0f);
        drawTextCentered(margin + content_w / 2.0, y, "!! UNDER ATTACK !!");
        y += line_h;
      }
    } else {
      drawTextDim(margin, y, "Robot: no data");
    }
    y += line_h + 4.0;

    // ========== RFID ==========
    drawSectionHeader(margin, y, content_w, "[ RFID ]",
                      {0.3f, 0.55f, 0.8f, 1.0f});
    y += line_h + 2.0;

    int rfid_count{0};
    if (d.rfid_status.base_gain_point) {
      drawRfidTag(margin, y, "BASE");
      y += line_h;
      ++rfid_count;
    }
    if (d.rfid_status.central_highland_gain_point) {
      drawRfidTag(margin, y, "CENTER HIGHLAND");
      y += line_h;
      ++rfid_count;
    }
    if (d.rfid_status.friendly_outpost_gain_point) {
      drawRfidTag(margin, y, "OUTPOST");
      y += line_h;
      ++rfid_count;
    }
    if (d.rfid_status.friendly_supply_zone_non_exchange
        || d.rfid_status.friendly_supply_zone_exchange) {
      drawRfidTag(margin, y, "SUPPLY ZONE");
      y += line_h;
      ++rfid_count;
    }
    if (d.rfid_status.friendly_big_resource_island) {
      drawRfidTag(margin, y, "RESOURCE ISLAND");
      y += line_h;
      ++rfid_count;
    }
    if (rfid_count == 0) {
      drawTextDim(margin, y, "(no RFID points active)");
    }
  }

 private:
  // ===== Drawing primitives =====

  void drawText(double x, double y, const std::string& text,
                const std::array<float, 4>& color = {1.0f, 1.0f, 1.0f,
                                                     1.0f}) {
    glColor4f(color[0], color[1], color[2], color[3]);
    pangolin::default_font().Text("%s", text.c_str()).Draw(x, y);
  }

  void drawTextDim(double x, double y, const std::string& text) {
    glColor4f(0.38f, 0.38f, 0.48f, 1.0f);
    pangolin::default_font().Text("%s", text.c_str()).Draw(x, y);
  }

  void drawTextRight(
      double right_x, double y, const std::string& text,
      const std::array<float, 4>& color = {1.0f, 1.0f, 1.0f, 1.0f}) {
    glColor4f(color[0], color[1], color[2], color[3]);
    // Approximate right-align: ~8px per character
    double approx_w = text.size() * 8.0;
    pangolin::default_font().Text("%s", text.c_str())
        .Draw(right_x - approx_w, y);
  }

  void drawTextCentered(double cx, double y, const std::string& text) {
    double approx_w = text.size() * 8.0;
    glColor4f(1.0f, 0.15f, 0.15f, 1.0f);
    pangolin::default_font().Text("%s", text.c_str())
        .Draw(cx - approx_w / 2.0, y);
  }

  void drawLabel(double x, double y, const std::string& label) {
    glColor4f(0.5f, 0.5f, 0.6f, 1.0f);
    pangolin::default_font().Text("%s", label.c_str()).Draw(x, y);
  }

  // ===== Panel frame =====

  void drawPanelBackground(double x, double y, double w, double h) {
    glColor4f(0.10f, 0.10f, 0.13f, 0.95f);
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();

    glColor4f(0.25f, 0.25f, 0.35f, 1.0f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();
  }

  void drawTitleBar(double x, double y, double w, double h,
                    const std::string& title,
                    const std::array<float, 4>& accent) {
    glColor4f(0.15f, 0.18f, 0.25f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();

    // Accent line at bottom of title bar
    glColor4f(accent[0], accent[1], accent[2], accent[3]);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glEnd();

    glColor4f(0.85f, 0.90f, 0.95f, 1.0f);
    pangolin::default_font().Text("%s", title.c_str())
        .Draw(x + 8.0, y + h - 20.0);
  }

  // ===== Content components =====

  void drawSectionHeader(double x, double y, double w, const std::string& label,
                         const std::array<float, 4>& color) {
    glColor4f(color[0], color[1], color[2], color[3]);
    pangolin::default_font().Text("%s", label.c_str()).Draw(x, y);

    glColor4f(0.18f, 0.18f, 0.25f, 1.0f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2f(x + 70.0, y - 2.0);
    glVertex2f(x + w, y - 2.0);
    glEnd();
  }

  void drawLabeledBar(double x, double y, double max_w, double h, float ratio,
                      const std::array<float, 4>& color,
                      const std::string& label) {
    drawLabel(x, y + 10.0, label);

    const double label_w{55.0};
    const double bar_x{x + label_w};
    const double bar_w{max_w - label_w - 60.0};

    // Background
    glColor4f(0.15f, 0.15f, 0.18f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2f(bar_x, y);
    glVertex2f(bar_x + bar_w, y);
    glVertex2f(bar_x + bar_w, y + h);
    glVertex2f(bar_x, y + h);
    glEnd();

    // Fill
    glColor4f(color[0], color[1], color[2], color[3]);
    const double fill_w = bar_w * static_cast<double>(ratio);
    glBegin(GL_QUADS);
    glVertex2f(bar_x, y);
    glVertex2f(bar_x + fill_w, y);
    glVertex2f(bar_x + fill_w, y + h);
    glVertex2f(bar_x, y + h);
    glEnd();

    // Border
    glColor4f(0.22f, 0.22f, 0.28f, 0.8f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(bar_x, y);
    glVertex2f(bar_x + bar_w, y);
    glVertex2f(bar_x + bar_w, y + h);
    glVertex2f(bar_x, y + h);
    glEnd();
  }

  void drawRfidTag(double x, double y, const std::string& label) {
    glColor4f(0.2f, 0.85f, 0.3f, 1.0f);
    glPointSize(6.0f);
    glBegin(GL_POINTS);
    glVertex2f(x + 4.0, y + 6.0);
    glEnd();

    glColor4f(0.5f, 0.9f, 0.5f, 1.0f);
    pangolin::default_font().Text(" %s", label.c_str()).Draw(x + 12.0, y);
  }

  // ===== Color helpers =====

  static std::array<float, 4> hpColor(float ratio) {
    if (ratio > 0.5f) return {0.2f, 0.9f, 0.3f, 1.0f};
    if (ratio > 0.2f) return {1.0f, 0.8f, 0.0f, 1.0f};
    return {1.0f, 0.2f, 0.2f, 1.0f};
  }

  static std::array<float, 4> heatColor(float ratio) {
    if (ratio < 0.3f) return {0.3f, 0.6f, 1.0f, 1.0f};
    if (ratio < 0.7f) return {1.0f, 0.55f, 0.0f, 1.0f};
    return {1.0f, 0.15f, 0.15f, 1.0f};
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

}  // namespace guga_ui

#endif // PANEL_HUD_HPP