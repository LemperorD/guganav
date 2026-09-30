#pragma once
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include "behaviortree_cpp_v3/bt_factory.h"

class SetGoalPose : public BT::SyncActionNode
{
public:
  SetGoalPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return{BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y")};
  }

  BT::NodeStatus tick() override
  {
    double goal_pose_x, goal_pose_y;
    if(!getInput<double>("pose_x",goal_pose_x)) {
      return BT::NodeStatus::FAILURE;
    }
    if(!getInput<double>("pose_y",goal_pose_y)) {
      return BT::NodeStatus::FAILURE;
    }

    auto bb = config().blackboard;
    bb->set("goal_pose_x", goal_pose_x);
    bb->set("goal_pose_y", goal_pose_y);

    return BT::NodeStatus::SUCCESS;
  }
};