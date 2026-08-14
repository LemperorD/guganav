#pragma once

#include <memory>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

namespace guga_ui
{
namespace cmdvel_visualizer
{

class CmdVelVisualizer
  : public rviz_common::RosTopicDisplay<geometry_msgs::msg::Twist>
{
  Q_OBJECT

public:
  CmdVelVisualizer();
  ~CmdVelVisualizer() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void processMessage(
    geometry_msgs::msg::Twist::ConstSharedPtr msg) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private:
  void createArrow();
  void createAngularRing();
  void createGroundDisc();
  void updateArrow(double vx, double vy);
  void updateAngularRing(double wz);
  void clearVisuals();

  Ogre::SceneNode* arrow_node_{nullptr};
  Ogre::ManualObject* arrow_object_{nullptr};
  Ogre::SceneNode* ring_node_{nullptr};
  Ogre::ManualObject* ring_object_{nullptr};
  Ogre::SceneNode* disc_node_{nullptr};
  Ogre::ManualObject* disc_object_{nullptr};

  rviz_common::properties::FloatProperty* scale_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* shaft_diameter_property_;
  rviz_common::properties::FloatProperty* head_size_property_;
  rviz_common::properties::ColorProperty* linear_color_property_;
  rviz_common::properties::ColorProperty* angular_color_property_;
  rviz_common::properties::FloatProperty* timeout_property_;
  rviz_common::properties::BoolProperty* show_angular_property_;

  double time_since_last_msg_{};
  bool has_message_{};
  double latest_vx_{};
  double latest_vy_{};
  double latest_wz_{};
};

}  // namespace cmdvel_visualizer
}  // namespace guga_ui
