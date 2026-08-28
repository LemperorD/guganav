#ifndef GUGA_UI_PANGOLIN_PANEL_HUD_HPP
#define GUGA_UI_PANGOLIN_PANEL_HUD_HPP

#include <cstdio>
#include <string>
#include "data_source.hpp"
#include "panel_const.hpp"
#include <pangolin/pangolin.h>
#include "guga_ui_common/shm_reader.hpp"
#include "guga_ui_common/ui_types.hpp"
namespace guga_ui {
  /// HUD 面板：显示比赛剩余时间。
  ///
  /// 注意：Pangolin 默认字体（AnonymousPro）不含中文字形，
  /// GlFont::Text() 对缺失字形会静默跳过，因此变量名必须使用 ASCII
  /// 字符，否则中文标签会渲染为空白。
  class PanelHud {
  public:
    PanelHud() = default;
    /// 创建 HUD 面板并注册“比赛时间”标签。
    void init(const UiDataSource& ui_data_source) {
      pangolin::CreatePanel("HUD").SetBounds(
          0.5, 1.0, 0.0, pangolin::Attach::Pix(UiPanelWidth()));
    }

    /// 每帧调用，刷新比赛状态。
    void update(const UiDataSource& ui_data_source) {
      const auto& d = ui_data_source.data();
      if (d.game_status_age > 0) {
        match_status_ = std::to_string(d.game_status.game_progress);
      } else {
        match_status_ = "--";
      }

      // 决策槽位从未写入时显示 "--"，避免把 0 误当成真实目标点。
      if (d.decision_age > 0) {
        target_x_ = FormatCoord(d.decision.target_x);
      } else {
        target_x_ = "--";
      }
    }

  private:
    /// 坐标格式化：保留两位小数，如 "1.50"。
    static std::string FormatCoord(double value) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.2f", value);
      return buf;
    }

    pangolin::Var<std::string> match_status_{"HUD.Match Status", "0"};
    pangolin::Var<std::string> target_x_{"HUD.Target Position X", "--"};
  };
}  // namespace guga_ui
#endif  // GUGA_UI_PANGOLIN_PANEL_HUD_HPP
