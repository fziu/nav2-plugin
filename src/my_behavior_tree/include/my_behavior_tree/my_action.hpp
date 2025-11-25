#ifndef MY_ACTION_HPP
#define MY_ACTION_HPP

#include <nav2_behavior_tree/bt_action_node.hpp>
#include "my_behavior_action/action/my_send.hpp"

namespace my_behavior_tree
{

using namespace nav2_behavior_tree;

class MyAction : public BtActionNode<my_behavior_action::action::MySend>
{
public:
    MyAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<bool>("flag", false, "start or stop")
        });
    }
};    

}

#endif  // MY_ACTION_HPP