#ifndef MY_BEHAVIOR_ACTION_HPP
#define MY_BEHAVIOR_ACTION_HPP

#include <nav2_behaviors/timed_behavior.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <my_behavior_action/action/my_send.hpp>

namespace my_behavior_action
{

using namespace nav2_behaviors;
using Action = my_behavior_action::action::MySend;

class MyBehaviorAction : public TimedBehavior<Action>
{
public:
    MyBehaviorAction();
    ~MyBehaviorAction();

    Status onRun(const std::shared_ptr<const Action::Goal> command) override;

    Status onCycleUpdate() override;

    void onConfigure() override;

protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Logger logger_{rclcpp::get_logger("MyBehaviorAction")};
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Time last_time;
    bool flag_;
};

}

#endif  // MY_BEHAVIOR_ACTION_HPP