#ifndef GUGA_UI_PANGOLIN_RENDER_3D_HPP
#define GUGA_UI_PANGOLIN_RENDER_3D_HPP
/**
 * @file render_3d.hpp
 * @brief 3D 场景渲染：机器人位姿、导航路径/目标点、敌方标记、坐标轴、地面参考网格。
 */

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/opengl_render_state.h>

#include "data_source.hpp"

/**
 * @brief 3D 视图渲染器。
 *
 * 使用 Pangolin 的 OpenGlRenderState 管理相机，
 * 支持顶视图（top-down）和跟随机器人两种模式。
 */
class GugaRender3D {
 public:
  /// 视图模式
  enum class ViewMode {
    TOP_DOWN,  // 俯视顶视图
    FOLLOW     // 跟随机器人
  };

  /**
   * @brief 初始化 3D 渲染器，设置默认相机参数。
   * @param view Pangolin View 对象引用（用于设置 handler）。
   */
  void init(pangolin::View& view) {
    // 默认顶视图：相机在(0,0,15)看向(0,0,0)，上方为y
    const double w = 1280.0;
    const double h = 800.0;

    s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(w, h, 500, 500, w / 2.0, h / 2.0, 0.1,
                                   100.0),
        pangolin::ModelViewLookAt(0.0, 0.0, 15.0,   // 相机位置
                                   0.0, 0.0, 0.0,    // 看向
                                   pangolin::AxisY)  // 上方为 Y
    );

    // 设置交互 handler（鼠标拖拽旋转/缩放/平移）
    view.SetHandler(&handler_);
  }

  /**
   * @brief 每帧绘制回调。
   * @param view Pangolin View 引用。
   * @param ds 数据源。
   */
  void draw(pangolin::View& view, const UiDataSource& ds) {
    view.Activate(s_cam_);

    // 清除背景为深灰
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const auto& d = ds.data();

    // ---- 1. 地面网格 ----
    drawGroundGrid();

    // ---- 2. 机器人位姿 ----
    if (ds.isFresh(d.odom_age)) {
      drawRobotPose(d.odom);
    }

    // ---- 3. 导航目标点 ----
    if (ds.isFresh(d.decision_age)) {
      drawNavGoal(d.decision);
    }

    // ---- 4. 导航路径 ----
    if (ds.isFresh(d.path_age) && d.path.count > 0) {
      drawPath(d.path);
    }

    // ---- 5. 敌方标记 ----
    if (ds.isFresh(d.enemy_age)) {
      drawEnemy(d.enemy);
    }

    // ---- 6. 无数据提示 ----
    if (!d.has_data) {
      drawNoData();
    }
  }

  /// 切换视图模式
  void toggleViewMode() {
    mode_ = (mode_ == ViewMode::TOP_DOWN) ? ViewMode::FOLLOW
                                           : ViewMode::TOP_DOWN;
  }

 private:
  // ---- 地面参考网格 ----

  void drawGroundGrid() {
    glLineWidth(1.0f);
    glColor4f(0.25f, 0.25f, 0.30f, 0.6f);

    // 10m × 10m 网格，1m 间距
    const float extent{10.0f};
    const float step{1.0f};

    glBegin(GL_LINES);
    for (float i = -extent; i <= extent; i += step) {
      // X 方向
      glVertex3f(i, -extent, 0.0f);
      glVertex3f(i, extent, 0.0f);
      // Y 方向
      glVertex3f(-extent, i, 0.0f);
      glVertex3f(extent, i, 0.0f);
    }
    glEnd();

    // X 轴: 红, Y 轴: 绿
    glLineWidth(2.0f);
    glColor4f(0.8f, 0.2f, 0.2f, 1.0f);  // X red
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(2.0f, 0.0f, 0.0f);
    glEnd();

    glColor4f(0.2f, 0.8f, 0.2f, 1.0f);  // Y green
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 2.0f, 0.0f);
    glEnd();

    // 原点
    glPointSize(6.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 0.8f);
    glBegin(GL_POINTS);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glEnd();
  }

  // ---- 机器人位姿 ----

  void drawRobotPose(const guga_ui::UiOdom& odom) {
    // 使用里程计数据绘制机器人位置和朝向
    const double x = odom.x;
    const double y = odom.y;
    const double yaw = odom.yaw;

    // 绘制三角朝向指示器
    const double size{0.3};
    const double cx = std::cos(yaw);
    const double sy = std::sin(yaw);

    glColor4f(0.2f, 0.7f, 1.0f, 1.0f);  // 浅蓝色三角
    glBegin(GL_TRIANGLES);
    glVertex3f(x + cx * size,        y + sy * size,        0.02f);
    glVertex3f(x - cx * size * 0.5f + sy * size * 0.5f,
               y - sy * size * 0.5f - cx * size * 0.5f, 0.02f);
    glVertex3f(x - cx * size * 0.5f - sy * size * 0.5f,
               y - sy * size * 0.5f + cx * size * 0.5f, 0.02f);
    glEnd();

    // 包围圆
    glColor4f(0.2f, 0.7f, 1.0f, 0.5f);
    glLineWidth(1.5f);
    drawCircleXY(x, y, size * 1.2f, 24);
  }

  // ---- 导航目标点 ----

  void drawNavGoal(const guga_ui::UiDecision& dec) {
    if (!dec.should_publish_goal) return;

    // 目标点十字
    const float rad{0.3f};
    glColor4f(1.0f, 0.6f, 0.1f, 1.0f);  // 橙色
    glLineWidth(2.0f);

    glBegin(GL_LINES);
    glVertex3f(dec.target_x - rad, dec.target_y, 0.03f);
    glVertex3f(dec.target_x + rad, dec.target_y, 0.03f);
    glVertex3f(dec.target_x, dec.target_y - rad, 0.03f);
    glVertex3f(dec.target_x, dec.target_y + rad, 0.03f);
    glEnd();

    // 目标朝向线
    const double len{0.5};
    glColor4f(1.0f, 0.6f, 0.1f, 0.6f);
    glBegin(GL_LINES);
    glVertex3f(dec.target_x, dec.target_y, 0.03f);
    glVertex3f(dec.target_x + len * std::cos(dec.target_yaw),
               dec.target_y + len * std::sin(dec.target_yaw), 0.03f);
    glEnd();
  }

  // ---- 导航路径 ----

  void drawPath(const guga_ui::UiPath& path) {
    glColor4f(0.0f, 0.8f, 0.4f, 0.7f);  // 绿色路径
    glLineWidth(2.0f);

    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < path.count && i < guga_ui::UI_PATH_MAX_POINTS;
         ++i) {
      glVertex3f(path.x[i], path.y[i], 0.02f);
    }
    glEnd();
  }

  // ---- 敌方标记 ----

  void drawEnemy(const guga_ui::UiEnemy& enemy) {
    // 追踪目标
    if (enemy.tracking) {
      glColor4f(1.0f, 0.2f, 0.2f, 0.9f);  // 红色
      drawCircleXY(enemy.target_x, enemy.target_y, 0.25f, 20);
    }

    // 攻击目标
    if (enemy.enemy_detected) {
      glColor4f(1.0f, 0.8f, 0.0f, 0.8f);  // 黄色十字
      glLineWidth(2.5f);
      const float dx{0.2f};
      glBegin(GL_LINES);
      glVertex3f(enemy.attack_x - dx, enemy.attack_y, 0.04f);
      glVertex3f(enemy.attack_x + dx, enemy.attack_y, 0.04f);
      glVertex3f(enemy.attack_x, enemy.attack_y - dx, 0.04f);
      glVertex3f(enemy.attack_x, enemy.attack_y + dx, 0.04f);
      glEnd();
    }
  }

  // ---- 工具 ----

  /// 在 XY 平面画圆
  void drawCircleXY(double cx, double cy, float radius, int segments) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
      const float angle = 2.0f * static_cast<float>(M_PI)
                        * static_cast<float>(i)
                        / static_cast<float>(segments);
      glVertex3f(cx + radius * std::cos(angle),
                 cy + radius * std::sin(angle), 0.03f);
    }
    glEnd();
  }

  /// 无数据时显示提示
  void drawNoData() {
    // NO DATA 文字通过 HUD 面板绘制，这里仅改变背景色调提示
    glClearColor(0.15f, 0.10f, 0.10f, 1.0f);
  }

  pangolin::OpenGlRenderState s_cam_;
  // pangolin::Handler3D handler_{pangolin::AxisNone};
  pangolin::Handler3D handler_{s_cam_};
  ViewMode mode_{ViewMode::TOP_DOWN};
};

#endif  // GUGA_UI_PANGOLIN_RENDER_3D_HPP
