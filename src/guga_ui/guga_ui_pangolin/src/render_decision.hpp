#ifndef GUGA_UI_PANGOLIN_RENDER_DECISION_HPP
#define GUGA_UI_PANGOLIN_RENDER_DECISION_HPP
/**
 * @file render_decision.hpp
 * @brief 决策面板渲染：当前决策状态、底盘模式、目标位姿、机器人当前位置。
 */

#include <cstdint>
#include <cstdio>
#include <string>

#include <pangolin/display/default_font.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include "data_source.hpp"

class GugaRenderDecision {
 public:
  void draw(pangolin::View& view, const UiDataSource& ds) {
    view.ActivatePixelOrthographic();
    glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    const auto& d = ds.data();
    double y{20.0};
    const double line_h{22.0};
    const double margin{15.0};

    // ---- 标题 ----
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    drawText(margin, y, "=== DECISION ===");
    y += line_h * 1.2;

    if (!ds.isFresh(d.decision_age, 120)) {
      drawTextDim(margin, y, "Decision: no data");
      return;
    }

    const auto& dec = d.decision;

    // ---- 决策状态 ----
    glColor4f(0.2f, 0.9f, 0.9f, 1.0f);  // cyan
    char buf[128];
    std::snprintf(buf, sizeof(buf), "State: %s", stateStr(dec.state));
    drawText(margin, y, buf);
    y += line_h;

    // ---- 底盘模式 ----
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    std::snprintf(buf, sizeof(buf), "Mode:  %s", chassisModeStr(dec.chassis_mode));
    drawText(margin, y, buf);
    y += line_h * 1.5;

    // ---- 当前位置 ----
    drawText(margin, y, "--- Robot Pose ---");
    y += line_h;

    std::snprintf(buf, sizeof(buf), "X: %7.3f m", dec.robot_x);
    drawText(margin, y, buf);
    y += line_h;

    std::snprintf(buf, sizeof(buf), "Y: %7.3f m", dec.robot_y);
    drawText(margin, y, buf);
    y += line_h;

    std::snprintf(buf, sizeof(buf), "Yaw: %5.1f deg", dec.robot_yaw * 180.0 / M_PI);
    drawText(margin, y, buf);
    y += line_h * 1.2;

    // ---- 目标位姿 ----
    if (dec.should_publish_goal) {
      glColor4f(1.0f, 0.6f, 0.1f, 1.0f);  // 橙色高亮
      drawText(margin, y, "--- Target Pose ---");
      y += line_h;

      std::snprintf(buf, sizeof(buf), "X: %7.3f m", dec.target_x);
      drawText(margin, y, buf);
      y += line_h;

      std::snprintf(buf, sizeof(buf), "Y: %7.3f m", dec.target_y);
      drawText(margin, y, buf);
      y += line_h;

      std::snprintf(buf, sizeof(buf), "Yaw: %5.1f deg",
                    dec.target_yaw * 180.0 / M_PI);
      drawText(margin, y, buf);
      y += line_h * 1.2;
    } else {
      drawTextDim(margin, y, "Goal: no target");
      y += line_h;
    }

    // ---- 云台 yaw ----
    if (ds.isFresh(d.yaw_age, 120)) {
      y += line_h * 0.5;
      drawText(margin, y, "--- Gimbal Yaw ---");
      y += line_h;

      std::snprintf(buf, sizeof(buf), "YawDiff: %5.1f deg",
                    d.yaw.yaw_diff * 180.0 / M_PI);
      drawText(margin, y, buf);
      y += line_h;

      std::snprintf(buf, sizeof(buf), "TES Wz: %6.3f rad/s",
                    d.yaw.tes_angular_z);
      drawText(margin, y, buf);
    }

    // ---- 决策耗时 ----
    if (dec.compute_us > 0) {
      y += line_h * 1.2;
      drawTextDim(margin, y,
                  "Compute: " + std::to_string(dec.compute_us) + " us");
    }
  }

 private:
  void drawText(double x, double y, const std::string& text) {
    pangolin::default_font().Text("%s", text.c_str()).Draw(x, y);
  }

  void drawTextDim(double x, double y, const std::string& text) {
    glColor4f(0.35f, 0.35f, 0.45f, 1.0f);
    pangolin::default_font().Text("%s", text.c_str()).Draw(x, y);
  }

  const char* stateStr(uint8_t state) {
    switch (state) {
      case 0: return "DEFAULT (cruising)";
      case 1: return "ATTACK (engaging)";
      case 2: return "SUPPLY (resupplying)";
      default: return "?";
    }
  }

  const char* chassisModeStr(uint8_t mode) {
    switch (mode) {
      case 0: return "CHASSIS_FOLLOWED";
      case 1: return "LITTLE_TES";
      case 2: return "GO_HOME";
      default: return "?";
    }
  }
};

#endif  // GUGA_UI_PANGOLIN_RENDER_DECISION_HPP
