#include "cmdvel_visualizer/cmdvel_visualizer.hpp"

#include <cmath>

namespace guga_ui
{
namespace cmdvel_visualizer
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int ARROW_DETAIL = 24;
constexpr int RING_DETAIL = 80;
constexpr double DEFAULT_SHAFT_DIAMETER = 0.08;
constexpr double DEFAULT_HEAD_SIZE = 1.0;
constexpr double MIN_VISIBLE_ARROW_LENGTH = 0.3;
constexpr double DEFAULT_SCALE = 2.0;
constexpr double DEFAULT_ALPHA = 0.9;
constexpr double DEFAULT_TIMEOUT = 0.5;
constexpr double DEFAULT_RING_RADIUS = 0.5;
constexpr double RING_TUBE_WIDTH = 0.04;
constexpr double GROUND_DISC_RADIUS = 0.15;
}  // namespace

CmdVelVisualizer::CmdVelVisualizer()
{
  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", DEFAULT_SCALE, "Length scale factor for velocity arrow.", this);
  scale_property_->setMin(0.1);
  shaft_diameter_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Diameter", DEFAULT_SHAFT_DIAMETER,
    "Thickness of the arrow shaft.", this, SLOT(queueRender()));
  shaft_diameter_property_->setMin(0.02);
  head_size_property_ = new rviz_common::properties::FloatProperty(
    "Head Size", DEFAULT_HEAD_SIZE,
    "Size multiplier for the arrow head.", this, SLOT(queueRender()));
  head_size_property_->setMin(0.3);
  head_size_property_->setMax(4.0);
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", DEFAULT_ALPHA, "Opacity of the visualization.", this,
    SLOT(queueRender()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);
  linear_color_property_ = new rviz_common::properties::ColorProperty(
    "Linear Color", QColor(0, 255, 0), "Color of the linear velocity arrow.",
    this, SLOT(queueRender()));
  angular_color_property_ = new rviz_common::properties::ColorProperty(
    "Angular Color", QColor(255, 128, 0),
    "Color of the angular velocity ring.", this, SLOT(queueRender()));
  timeout_property_ = new rviz_common::properties::FloatProperty(
    "Timeout", DEFAULT_TIMEOUT,
    "Seconds after last message before visuals disappear.", this);
  timeout_property_->setMin(0.05);
  timeout_property_->setMax(5.0);
  show_angular_property_ = new rviz_common::properties::BoolProperty(
    "Show Angular", true, "Show angular velocity visualization.", this,
    SLOT(queueRender()));
}

CmdVelVisualizer::~CmdVelVisualizer() = default;

void CmdVelVisualizer::onInitialize()
{
  RTDClass::onInitialize();

  arrow_node_ = scene_node_->createChildSceneNode();
  ring_node_ = scene_node_->createChildSceneNode();
  disc_node_ = scene_node_->createChildSceneNode();

  createArrow();
  createAngularRing();
  createGroundDisc();
}

void CmdVelVisualizer::onEnable()
{
  arrow_node_->setVisible(true);
  ring_node_->setVisible(true);
  disc_node_->setVisible(true);
}

void CmdVelVisualizer::onDisable()
{
  arrow_node_->setVisible(false);
  ring_node_->setVisible(false);
  disc_node_->setVisible(false);
  reset();
}

void CmdVelVisualizer::reset()
{
  RTDClass::reset();
  has_message_ = false;
  time_since_last_msg_ = {};
  clearVisuals();
}

void CmdVelVisualizer::processMessage(
  geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  latest_vx_ = msg->linear.x;
  latest_vy_ = msg->linear.y;
  latest_wz_ = msg->angular.z;
  has_message_ = true;
  time_since_last_msg_ = {};

  updateArrow(latest_vx_, latest_vy_);
  updateAngularRing(latest_wz_);
  queueRender();
}

void CmdVelVisualizer::update(float wall_dt, float /*ros_dt*/)
{
  if (!has_message_) {
    return;
  }
  time_since_last_msg_ += static_cast<double>(wall_dt);
  if (time_since_last_msg_ > timeout_property_->getFloat()) {
    has_message_ = false;
    time_since_last_msg_ = {};
    clearVisuals();
    queueRender();
  }
}

void CmdVelVisualizer::createArrow()
{
  static const std::string kArrowName = "CmdVelArrow";
  arrow_object_ =
    scene_manager_->createManualObject(kArrowName);
  arrow_object_->setDynamic(true);
  arrow_node_->attachObject(arrow_object_);
}

void CmdVelVisualizer::createAngularRing()
{
  static const std::string kRingName = "CmdVelAngularRing";
  ring_object_ =
    scene_manager_->createManualObject(kRingName);
  ring_object_->setDynamic(true);
  ring_node_->attachObject(ring_object_);
}

void CmdVelVisualizer::createGroundDisc()
{
  static const std::string kDiscName = "CmdVelGroundDisc";
  disc_object_ =
    scene_manager_->createManualObject(kDiscName);
  disc_object_->setDynamic(true);
  disc_node_->attachObject(disc_object_);
}

void CmdVelVisualizer::updateArrow(double vx, double vy)
{
  double speed = std::hypot(vx, vy);
  double scale = scale_property_->getFloat();
  double sr = shaft_diameter_property_->getFloat() * 0.5;
  double head_mult = head_size_property_->getFloat();

  double length = speed * scale;
  if (length < MIN_VISIBLE_ARROW_LENGTH) {
    length = MIN_VISIBLE_ARROW_LENGTH;
  }

  double angle = std::atan2(vy, vx);
  if (speed < 1e-6) {
    angle = 0.0;
  }

  auto [r, g, b] = [&]() -> std::tuple<float, float, float> {
    QColor c = linear_color_property_->getColor();
    return {static_cast<float>(c.redF()),
            static_cast<float>(c.greenF()),
            static_cast<float>(c.blueF())};
  }();
  float a = alpha_property_->getFloat();

  arrow_object_->clear();
  Ogre::ColourValue color{r, g, b, a};
  Ogre::ColourValue color_faint{r, g, b, a * 0.35f};

  double hr = sr * 3.0 * head_mult;
  double hl = sr * 6.0 * head_mult;

  double shaft_len = length - hl;
  if (shaft_len < 0.0) {
    shaft_len = 0.0;
  }

  double head_base = shaft_len;

  arrow_object_->begin("BaseWhiteNoLighting",
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for (int i{}; i <= ARROW_DETAIL; ++i) {
    double theta = 2.0 * kPi * static_cast<double>(i)
                   / static_cast<double>(ARROW_DETAIL);
    double cx = std::cos(theta);
    double sy = std::sin(theta);

    double nx = std::cos(angle) * cx;
    double ny = std::sin(angle) * cx;

    arrow_object_->position(0.0, 0.0, sr * sy);
    arrow_object_->normal(nx, ny, sr * sy);
    arrow_object_->colour(color);

    arrow_object_->position(
      head_base * std::cos(angle),
      head_base * std::sin(angle), sr * sy);
    arrow_object_->normal(nx, ny, sr * sy);
    arrow_object_->colour(color);
  }

  for (int i{}; i < ARROW_DETAIL; ++i) {
    int b0 = i * 2;
    int b1 = b0 + 1;
    int n0 = (b0 + 2) % (ARROW_DETAIL * 2);
    int n1 = (b0 + 3) % (ARROW_DETAIL * 2);

    arrow_object_->triangle(b0, b1, n0);
    arrow_object_->triangle(b1, n1, n0);
  }

  double tip_x = length * std::cos(angle);
  double tip_y = length * std::sin(angle);
  double base_x = head_base * std::cos(angle);
  double base_y = head_base * std::sin(angle);

  int head_vertex_offset = (ARROW_DETAIL + 1) * 2;

  for (int i{}; i <= ARROW_DETAIL; ++i) {
    double theta = 2.0 * kPi * static_cast<double>(i)
                   / static_cast<double>(ARROW_DETAIL);
    double sy = std::sin(theta);
    double nx = std::cos(angle);

    arrow_object_->position(base_x, base_y, hr * sy);
    arrow_object_->normal(nx, 0.0, hr * sy);
    arrow_object_->colour(color);
  }

  int tip_idx = head_vertex_offset + (ARROW_DETAIL + 1);
  arrow_object_->position(tip_x, tip_y, 0.0);
  arrow_object_->normal(std::cos(angle), std::sin(angle), 0.0);
  arrow_object_->colour(color);

  for (int i{}; i < ARROW_DETAIL; ++i) {
    int b = head_vertex_offset + i;
    int n = head_vertex_offset + i + 1;
    arrow_object_->triangle(b, n, tip_idx);
  }

  int cap_offset = tip_idx + 1;
  arrow_object_->position(base_x, base_y, 0.0);
  arrow_object_->normal(-std::cos(angle), -std::sin(angle), 0.0);
  arrow_object_->colour(color);

  for (int i{}; i <= ARROW_DETAIL; ++i) {
    double theta = 2.0 * kPi * static_cast<double>(i)
                   / static_cast<double>(ARROW_DETAIL);
    arrow_object_->position(base_x, base_y, hr * std::sin(theta));
    arrow_object_->normal(-std::cos(angle), -std::sin(angle), 0.0);
    arrow_object_->colour(color);
  }

  int base_center_idx = cap_offset;
  for (int i{}; i < ARROW_DETAIL; ++i) {
    int b = cap_offset + 1 + i;
    int n = cap_offset + 1 + i + 1;
    arrow_object_->triangle(base_center_idx, n, b);
  }

  arrow_object_->end();

  disc_object_->clear();
  disc_object_->begin("BaseWhiteNoLighting",
                       Ogre::RenderOperation::OT_TRIANGLE_LIST);

  double dr = GROUND_DISC_RADIUS;
  int disc_center = 0;
  disc_object_->position(0.0, 0.0, 0.01);
  disc_object_->normal(0.0, 0.0, 1.0);
  disc_object_->colour(color_faint);

  for (int i{}; i <= ARROW_DETAIL; ++i) {
    double theta = 2.0 * kPi * static_cast<double>(i)
                   / static_cast<double>(ARROW_DETAIL);
    disc_object_->position(dr * std::cos(theta), dr * std::sin(theta), 0.01);
    disc_object_->normal(0.0, 0.0, 1.0);
    disc_object_->colour(color_faint);
  }

  for (int i{}; i < ARROW_DETAIL; ++i) {
    int b = 1 + i;
    int n = 1 + i + 1;
    disc_object_->triangle(disc_center, n, b);
  }

  disc_object_->end();
}

void CmdVelVisualizer::updateAngularRing(double wz)
{
  if (!show_angular_property_->getBool()) {
    ring_object_->clear();
    return;
  }

  auto [r, g, b] = [&]() -> std::tuple<float, float, float> {
    QColor c = angular_color_property_->getColor();
    return {static_cast<float>(c.redF()),
            static_cast<float>(c.greenF()),
            static_cast<float>(c.blueF())};
  }();
  float a = alpha_property_->getFloat();

  static constexpr double kMaxWz = 6.283;

  double frac = std::clamp(std::abs(wz) / kMaxWz, 0.0, 1.0);
  if (frac < 0.01) {
    ring_object_->clear();
    return;
  }

  double sweep = frac * 2.0 * kPi;
  bool is_cw = wz < 0.0;

  ring_object_->clear();
  ring_object_->begin("BaseWhiteNoLighting",
      Ogre::RenderOperation::OT_TRIANGLE_LIST);
  Ogre::ColourValue color{r, g, b, a};

  double rr = DEFAULT_RING_RADIUS;
  double tw = RING_TUBE_WIDTH;

  double s_start = is_cw ? -sweep : 0.0;
  double s_end = is_cw ? 0.0 : sweep;

  int segments = static_cast<int>(
    static_cast<double>(RING_DETAIL) * frac);
  if (segments < 6) {
    segments = 6;
  }

  for (int i{}; i <= segments; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(segments);
    double angle_rad = s_start + t * (s_end - s_start);
    double ca = std::cos(angle_rad);
    double sa = std::sin(angle_rad);

    double ox = rr * ca;
    double oy = rr * sa;

    double in_r = rr - tw;
    double ix = in_r * ca;
    double iy = in_r * sa;

    ring_object_->position(ox, oy, 0.02);
    ring_object_->normal(0.0, 0.0, 1.0);
    ring_object_->colour(color);

    ring_object_->position(ix, iy, 0.02);
    ring_object_->normal(0.0, 0.0, 1.0);
    ring_object_->colour(color);
  }

  for (int i{}; i < segments; ++i) {
    int o0 = i * 2;
    int i0 = o0 + 1;
    int o1 = o0 + 2;
    int i1 = o0 + 3;
    ring_object_->triangle(o0, o1, i0);
    ring_object_->triangle(o1, i1, i0);
  }

  ring_object_->end();
}

void CmdVelVisualizer::clearVisuals()
{
  arrow_object_->clear();
  ring_object_->clear();
  disc_object_->clear();
}

}  // namespace cmdvel_visualizer
}  // namespace guga_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  guga_ui::cmdvel_visualizer::CmdVelVisualizer,
  rviz_common::Display)
