#pragma once
#include "behaviortree_cpp_v3/bt_factory.h"

class CheckGreaterThan200 : public BT::SyncActionNode
{
public:
  CheckGreaterThan200(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  // 空缺一:声明两个端口
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("HP"), BT::OutputPort<bool>("result")};
  }

  // 空缺二:执行逻辑
  BT::NodeStatus tick() override
  {
    double hitpoint;

    if(!getInput<double>("HP",hitpoint))
    {
      return BT::NodeStatus::FAILURE;
    }

    bool result = (hitpoint > 200);
    
    auto bb = config().blackboard;
    bb->set("is_safe", result);
    
    return BT::NodeStatus::SUCCESS;
  }
};