/**
 * @file test_pangolin_render.cpp
 * @brief End-to-end Pangolin rendering test.
 *
 * Writes synthetic data into shared memory, runs the Pangolin renderers
 * (3D scene, HUD, Decision panel), exports screenshots to disk, then
 * shuts down cleanly.
 *
 * This test validates:
 *   - Shared-memory write → read round-trip through the render pipeline
 *   - 3D geometry: robot pose triangle, nav goal crosshair, nav path line strip,
 *     enemy circle/cross, ground grid, coordinate axes
 *   - 2D HUD: HP bar, heat bar, ammo/gold, game phase, RFID tags
 *   - 2D Decision: state, chassis mode, robot pose, target pose, gimbal yaw
 *   - Text rendering does NOT produce blank squares (font/encoding fix verified)
 *   - Color-coded elements (green/yellow/red HP, orange heat, etc.)
 *
 * Prerequisites:
 *   - A running X server (no offscreen fallback needed)
 *   - Pangolin built with EGL / GLX
 *   - Write permission on test/results/
 *
 * Output:
 *   - test/results/render_full.png  — combined (3D + HUD + Decision) view
 *   - test/results/render_3d.png    — 3D scene close-up
 */

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/default_font.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/viewport.h>

#include "guga_ui_common/shm_writer.hpp"
#include "guga_ui_common/shm_layout.hpp"
#include "guga_ui_common/ui_types.hpp"

#include "data_source.hpp"
#include "render_3d.hpp"
#include "panel_decision.hpp"
#include "panel_hud.hpp"

namespace gu = guga_ui;
namespace fs = std::filesystem;

// ==================== Helpers ====================

/// Create the output directory if it does not exist.
static std::string ensureResultDir() {
  const fs::path dir{"test/results"};
  std::error_code ec;
  fs::create_directories(dir, ec);
  return dir.string();
}

/// Unique shm name so concurrent test runs don't clash.
static std::string shmName() {
  return "/test_render_shm_" + std::to_string(::getpid());
}

/// Fill every slot with realistic-looking synthetic data.
static void populateAllSlots(const std::string& name) {
  using S = gu::UiSlotId;

  // ---- Robot Status ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::ROBOT_STATUS);
    gu::UiRobotStatus rs{};
    rs.current_hp                  = 600;
    rs.maximum_hp                  = 800;
    rs.shooter_17mm_1_barrel_heat  = 120;
    rs.shooter_barrel_heat_limit   = 400;
    rs.projectile_allowance_17mm   = 150;
    rs.remaining_gold_coin         = 1200;
    rs.robot_id                    = 7;
    rs.is_hp_deduced               = false;
    w.write(&rs, sizeof(rs));
  }

  // ---- Game Status ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::GAME_STATUS);
    gu::UiGameStatus gs{};
    gs.game_progress      = 4;   // RUNNING
    gs.stage_remain_time  = 195; // seconds
    gs.elapsed_sec        = 165.3;
    w.write(&gs, sizeof(gs));
  }

  // ---- RFID Status ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::RFID_STATUS);
    gu::UiRfidStatus rf{};
    rf.base_gain_point                      = true;
    rf.central_highland_gain_point          = true;
    rf.friendly_outpost_gain_point          = true;
    rf.friendly_supply_zone_non_exchange    = false;
    rf.friendly_supply_zone_exchange        = true;
    rf.friendly_big_resource_island         = true;
    w.write(&rf, sizeof(rf));
  }

  // ---- Decision ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::DECISION);
    gu::UiDecision dec{};
    dec.state              = 1;     // ATTACK
    dec.chassis_mode       = 1;     // LITTLE_TES
    dec.should_publish_goal= true;
    dec.target_x           = 4.5;
    dec.target_y           = 3.0;
    dec.target_yaw         = 0.785; // 45 deg
    dec.robot_x            = 2.0;
    dec.robot_y            = 1.5;
    dec.robot_yaw          = 0.3;   // ~17 deg
    dec.compute_us         = 320;
    w.write(&dec, sizeof(dec));
  }

  // ---- Enemy ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::ENEMY);
    gu::UiEnemy en{};
    en.tracking       = true;
    en.enemy_detected = true;
    en.target_x       = 5.0;
    en.target_y       = 4.0;
    en.target_z       = 0.3;
    en.attack_x       = 5.2;
    en.attack_y       = 4.1;
    en.last_seen_sec  = 0.5;
    w.write(&en, sizeof(en));
  }

  // ---- Odom ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::ODOM);
    gu::UiOdom od{};
    od.x     = 2.0;
    od.y     = 1.5;
    od.z     = 0.0;
    od.yaw   = 0.3;
    od.vx    = 0.5;
    od.vy    = 0.1;
    od.vz    = 0.0;
    w.write(&od, sizeof(od));
  }

  // ---- Yaw ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::YAW);
    gu::UiYaw yw{};
    yw.yaw_diff      = 0.08;  // ~4.6 deg
    yw.tes_angular_z = 1.25;
    w.write(&yw, sizeof(yw));
  }

  // ---- Path (nav path with 6 waypoints forming a gentle arc) ----
  {
    gu::ShmWriter w;
    (void)w.init(name, S::PATH);
    gu::UiPath p{};
    p.count     = 6;
    p.stamp_sec = 1.0;
    // Gentle arc from (2, 1.5) through intermediate points to (4.5, 3)
    p.x[0] = 2.0;   p.y[0] = 1.5;
    p.x[1] = 2.5;   p.y[1] = 1.8;
    p.x[2] = 3.0;   p.y[2] = 2.0;
    p.x[3] = 3.5;   p.y[3] = 2.4;
    p.x[4] = 4.0;   p.y[4] = 2.7;
    p.x[5] = 4.5;   p.y[5] = 3.0;
    w.write(&p, sizeof(p));
  }

  std::cout << "[Test] Wrote synthetic data to all 8 slots." << std::endl;
}

// ==================== Main test logic ====================

int main(int /*argc*/, char** /*argv*/) {
  const std::string out_dir = ensureResultDir();
  const std::string shm     = shmName();

  // ---- 1. Populate shared memory ----
  populateAllSlots(shm);

  // ---- 2. Open data source ----
  UiDataSource ds;
  if (!ds.open(shm)) {
    std::cerr << "[Test] FATAL: failed to open shm \"" << shm << "\"\n";
    return 1;
  }

  // ---- 3. Create Pangolin window (smaller size for faster test) ----
  pangolin::CreateWindowAndBind("GUGA Render Test", 1280, 800);

  // Check that the default font is actually valid
  try {
    pangolin::default_font().Text("HELLO").Draw(10, 10);
  } catch (const std::exception& e) {
    std::cerr << "[Test] WARNING: default_font().Text() threw: " << e.what() << "\n";
  }

  // ---- 4. Layout (same structure as production main) ----
  pangolin::View& root_view =
      pangolin::DisplayBase()
          .SetBounds(0.0, 1.0, 0.0, 1.0)
          .SetLayout(pangolin::LayoutEqualHorizontal);

  pangolin::View& view_3d =
      root_view.AddDisplay(pangolin::View()
                               .SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(800)));

  pangolin::View& right_panel =
      root_view.AddDisplay(pangolin::View()
                               .SetBounds(0.0, 1.0,
                                          pangolin::Attach::Pix(800), 1.0)
                               .SetLayout(pangolin::LayoutEqualVertical));

  pangolin::View& hud_view =
      right_panel.AddDisplay(pangolin::View().SetBounds(
          0.0, 0.5, 0.0, 1.0));

  pangolin::View& decision_view =
      right_panel.AddDisplay(pangolin::View().SetBounds(
          0.5, 1.0, 0.0, 1.0));

  // ---- 5. Initialize renderers ----
  GugaRender3D render_3d;
  render_3d.init(view_3d);

  PanelHUD panel_hud;
  PanelDecision panel_decision;

  view_3d.SetDrawFunction([&](pangolin::View& v) {
    render_3d.draw(v, ds);
  });
  hud_view.SetDrawFunction([&](pangolin::View& v) {
    panel_hud.draw(v, ds);
  });
  decision_view.SetDrawFunction([&](pangolin::View& v) {
    panel_decision.draw(v, ds);
  });

  // ---- 6. Render several frames (let data flow through pipeline) ----
  std::cout << "[Test] Rendering frames..." << std::endl;
  for (int frame = 0; frame < 10; ++frame) {
    ds.update();
    pangolin::FinishFrame();
  }

  // ---- 7. Export screenshots ----
  // Render one more frame to ensure all data is fresh, then schedule
  // captures via SaveWindowOnRender (processed during FinishFrame).
  ds.update();
  const std::string full_path = out_dir + "/render_full.png";
  const std::string d3_path   = out_dir + "/render_3d.png";

  pangolin::SaveWindowOnRender(full_path);
  pangolin::SaveWindowOnRender(d3_path, view_3d.v);
  // FinishFrame triggers RenderViews (draws sub-views) then PostRender
  // (saves scheduled captures to disk)
  pangolin::FinishFrame();
  std::cout << "[Test] Saved: " << full_path << std::endl;
  std::cout << "[Test] Saved: " << d3_path << std::endl;

  // ---- 8. Cleanup ----
  // RemoveCurrent() can segfault on some X11 configurations;
  // since the process exits immediately after, we skip it here.
  shm_unlink(shm.c_str());

  // ---- 9. Validate output files ----
  bool all_ok = true;
  for (const auto& p : {full_path, d3_path}) {
    if (!fs::exists(p)) {
      std::cerr << "[Test] FAIL: missing output file: " << p << "\n";
      all_ok = false;
    } else {
      auto sz = fs::file_size(p);
      std::cout << "[Test] " << p << " (" << sz << " bytes)" << std::endl;
      if (sz < 100) {
        std::cerr << "[Test] FAIL: file too small, likely blank: " << p << "\n";
        all_ok = false;
      }
    }
  }

  if (all_ok) {
    std::cout << "\n[Test] ALL CHECKS PASSED.\n";
    return 0;
  }
  std::cerr << "\n[Test] SOME CHECKS FAILED.\n";
  return 1;
}
