#include "my_behavior_tree/my_action.hpp"

namespace my_behavior_tree
{

MyAction::MyAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
: BtActionNode<my_behavior_action::action::MySend>(xml_tag_name, action_name, conf)
{
    bool flag_;
    getInput("flag", flag_);
    goal_.flag = flag_;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<my_behavior_tree::MyAction>(name, "MyAction", config);
    };

  factory.registerBuilder<my_behavior_tree::MyAction>("MyAction", builder);
}