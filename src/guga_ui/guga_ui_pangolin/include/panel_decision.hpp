#ifndef PANEL_DECISION_HPP
#define PANEL_DECISION_HPP
/**
 * @file panel_decision.hpp
 * @brief Decision panel renderer with panel aesthetic.
 *
 * Draws a bordered panel with title bar in pixel coordinates,
 * showing decision state, chassis mode, robot pose, nav goal,
 * gimbal yaw, and compute time.
 */

#include <cstdint>
#include <cstdio>
#include <string>

#include <pangolin/display/default_font.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include "data_source.hpp"

class PanelDecision {
 public:
  void draw(pangolin::View& view, const UiDataSource& ds) {
    view.ActivatePixelOrthographic();
    glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
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
    drawTitleBar(pad, pad, panel_w - pad * 2, title_h, "DECISION",
                 {1.0f, 0.5f, 0.1f, 1.0f});

    // Content area
    double y{pad + title_h + 8.0};
    const double line_h{20.0};
    const double margin{pad + 10.0};
    const double content_w{panel_w - pad * 2 - 20.0};

    if (!ds.isFresh(d.decision_age, 120)) {
      drawTextDim(margin, y, "Decision: no data");
      return;
    }

    const auto& dec = d.decision;

    // ========== MODE ==========
    drawSectionHeader(margin, y, content_w, "[ MODE ]",
                      {0.8f, 0.45f, 0.15f, 1.0f});
    y += line_h + 2.0;

    {
      char buf[64];
      const auto sc = stateColor(dec.state);
      std::snprintf(buf, sizeof(buf), "State: %s", stateStr(dec.state));
      drawText(margin, y, buf, sc);
      y += line_h;
    }

    {
      char buf[64];
      const auto mc = chassisColor(dec.chassis_mode);
      std::snprintf(buf, sizeof(buf), "Chassis: %s",
                    chassisModeStr(dec.chassis_mode));
      drawText(margin, y, buf, mc);
    }
    y += line_h + 4.0;

    // ========== ROBOT POSE ==========
    drawSectionHeader(margin, y, content_w, "[ ROBOT POSE ]",
                      {0.8f, 0.45f, 0.15f, 1.0f});
    y += line_h + 2.0;

    {
      char buf[64];
      drawLabel(margin, y, "X");
      std::snprintf(buf, sizeof(buf), "%7.3f m", dec.robot_x);
      drawTextRightLabel(margin + 40.0, y, buf);
      y += line_h;

      drawLabel(margin, y, "Y");
      std::snprintf(buf, sizeof(buf), "%7.3f m", dec.robot_y);
      drawTextRightLabel(margin + 40.0, y, buf);
      y += line_h;

      drawLabel(margin, y, "Yaw");
      double deg = dec.robot_yaw * 180.0 / M_PI;
      while (deg > 180.0) deg -= 360.0;
      while (deg < -180.0) deg += 360.0;
      std::snprintf(buf, sizeof(buf), "%6.1f deg", deg);
      drawTextRightLabel(margin + 40.0, y, buf);
    }
    y += line_h + 4.0;

    // ========== TARGET ==========
    drawSectionHeader(margin, y, content_w, "[ TARGET ]",
                      {0.8f, 0.45f, 0.15f, 1.0f});
    y += line_h + 2.0;

    if (dec.should_publish_goal) {
      char buf[64];
      const auto gc = std::array<float, 4>{1.0f, 0.6f, 0.1f, 1.0f};

      drawLabel(margin, y, "X");
      std::snprintf(buf, sizeof(buf), "%7.3f m", dec.target_x);
      drawTextRightLabel(margin + 40.0, y, buf, gc);
      y += line_h;

      drawLabel(margin, y, "Y");
      std::snprintf(buf, sizeof(buf), "%7.3f m", dec.target_y);
      drawTextRightLabel(margin + 40.0, y, buf, gc);
      y += line_h;

      drawLabel(margin, y, "Yaw");
      double tdeg = dec.target_yaw * 180.0 / M_PI;
      while (tdeg > 180.0) tdeg -= 360.0;
      while (tdeg < -180.0) tdeg += 360.0;
      std::snprintf(buf, sizeof(buf), "%6.1f deg", tdeg);
      drawTextRightLabel(margin + 40.0, y, buf, gc);

      // Active goal indicator dot
      const double dot_x = margin + content_w - 10.0;
      glColor4f(1.0f, 0.5f, 0.0f, 1.0f);
      glPointSize(8.0f);
      glBegin(GL_POINTS);
      glVertex2f(dot_x, y + 2.0);
      glEnd();
      drawText(dot_x - 22.0, y, "GOAL", {1.0f, 0.5f, 0.0f, 1.0f});
    } else {
      drawTextDim(margin, y, "(no target set)");
    }
    y += line_h + 4.0;

    // ========== GIMBAL ==========
    if (ds.isFresh(d.yaw_age, 120)) {
      drawSectionHeader(margin, y, content_w, "[ GIMBAL ]",
                        {0.8f, 0.45f, 0.15f, 1.0f});
      y += line_h + 2.0;

      char buf[64];

      drawLabel(margin, y, "Yaw Diff");
      double yd = d.yaw.yaw_diff * 180.0 / M_PI;
      while (yd > 180.0) yd -= 360.0;
      while (yd < -180.0) yd += 360.0;
      std::snprintf(buf, sizeof(buf), "%6.1f deg", yd);
      drawTextRightLabel(margin + 65.0, y, buf, yawDiffColor(std::abs(yd)));
      y += line_h;

      drawLabel(margin, y, "TES Wz");
      std::snprintf(buf, sizeof(buf), "%7.3f rad/s", d.yaw.tes_angular_z);
      drawTextRightLabel(margin + 65.0, y, buf);
    } else {
      drawTextDim(margin, y + line_h / 2.0, "Gimbal: no data");
    }
    y += line_h + 4.0;

    // ========== COMPUTE TIME (footer) ==========
    if (dec.compute_us > 0) {
      const double footer_y = panel_h - pad - 22.0;
      glColor4f(0.15f, 0.15f, 0.22f, 1.0f);
      glLineWidth(1.0f);
      glBegin(GL_LINES);
      glVertex2f(margin, footer_y);
      glVertex2f(margin + content_w, footer_y);
      glEnd();

      char buf[32];
      std::snprintf(buf, sizeof(buf), "compute: %ld us", dec.compute_us);
      drawTextDim(margin, footer_y + 14.0, buf);
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

  void drawLabel(double x, double y, const std::string& label) {
    glColor4f(0.5f, 0.5f, 0.6f, 1.0f);
    pangolin::default_font().Text("%s", label.c_str()).Draw(x, y);
  }

  void drawTextRightLabel(
      double x, double y, const std::string& text,
      const std::array<float, 4>& color = {1.0f, 1.0f, 1.0f, 1.0f}) {
    glColor4f(color[0], color[1], color[2], color[3]);
    pangolin::default_font().Text("%s", text.c_str()).Draw(x, y);
  }

  // ===== Panel frame =====

  void drawPanelBackground(double x, double y, double w, double h) {
    glColor4f(0.09f, 0.09f, 0.12f, 0.95f);
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();

    glColor4f(0.22f, 0.22f, 0.32f, 1.0f);
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
    glColor4f(0.13f, 0.15f, 0.22f, 1.0f);
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

    glColor4f(0.16f, 0.16f, 0.22f, 1.0f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2f(x + 70.0, y - 2.0);
    glVertex2f(x + w, y - 2.0);
    glEnd();
  }

  // ===== Color and string helpers =====

  static std::array<float, 4> stateColor(uint8_t state) {
    switch (state) {
      case 0: return {0.6f, 0.6f, 0.6f, 1.0f};
      case 1: return {0.2f, 0.9f, 0.4f, 1.0f};
      case 2: return {0.2f, 0.6f, 1.0f, 1.0f};
      default: return {1.0f, 1.0f, 1.0f, 1.0f};
    }
  }

  static std::array<float, 4> chassisColor(uint8_t mode) {
    switch (mode) {
      case 0: return {0.5f, 0.8f, 1.0f, 1.0f};
      case 1: return {1.0f, 0.7f, 0.2f, 1.0f};
      case 2: return {0.8f, 0.4f, 0.8f, 1.0f};
      default: return {1.0f, 1.0f, 1.0f, 1.0f};
    }
  }

  static std::array<float, 4> yawDiffColor(double abs_deg) {
    if (abs_deg < 5.0) return {0.3f, 0.9f, 0.4f, 1.0f};
    if (abs_deg < 20.0) return {1.0f, 0.8f, 0.1f, 1.0f};
    return {1.0f, 0.3f, 0.3f, 1.0f};
  }

  const char* stateStr(uint8_t state) {
    switch (state) {
      case 0: return "DEFAULT";
      case 1: return "ATTACK";
      case 2: return "SUPPLY";
      default: return "?";
    }
  }

  const char* chassisModeStr(uint8_t mode) {
    switch (mode) {
      case 0: return "FOLLOWED";
      case 1: return "LITTLE_TES";
      case 2: return "GO_HOME";
      default: return "?";
    }
  }
};

#endif // PANEL_DECISION_HPP