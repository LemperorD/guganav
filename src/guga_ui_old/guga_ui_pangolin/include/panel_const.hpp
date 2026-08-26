#ifndef PANEL_CONST_HPP
#define PANEL_CONST_HPP

#include <pangolin/display/default_font.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

/// 面板宽度（像素）。
///
/// 注意：必须延迟到 pangolin::CreateWindowAndBind() 之后调用，
/// 因为 default_font() 依赖当前 GL context（Pangolin 内部 PANGO_ASSERT(context)）。
/// 若作为全局变量初始化表达式调用，会在 main() 之前 abort。
inline int UiPanelWidth() {
  return static_cast<int>(20 * pangolin::default_font().MaxWidth());
}

#endif // PANEL_CONST_HPP
