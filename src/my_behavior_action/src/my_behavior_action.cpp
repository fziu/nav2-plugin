#include "my_behavior_action/my_behavior_action.hpp"

namespace my_behavior_action
{

MyBehaviorAction::MyBehaviorAction() : TimedBehavior<Action>(), flag_(false) {}

MyBehaviorAction::~MyBehaviorAction() {}

void MyBehaviorAction::onConfigure()
{
    node = node_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
}

Status MyBehaviorAction::onRun(const std::shared_ptr<const Action::Goal> command)
{
    flag_ = command->flag;
    last_time = clock_->now();
    return Status::SUCCEEDED;
}

Status MyBehaviorAction::onCycleUpdate()
{
    if (!flag_) return Status::FAILED;

    geometry_msgs::msg::Twist msg;

    int delta_time = static_cast<int>((clock_->now() - last_time).seconds());
    if (delta_time > 10) {
        vel_pub_->publish(msg);
        return Status::SUCCEEDED;
    }

    msg.linear.x = 0.0;
    if (delta_time % 2 == 0) {
        msg.angular.z = 1.5;
    } else {
        msg.angular.z = -1.5;
    }

    vel_pub_->publish(msg);
    return Status::RUNNING;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_behavior_action::MyBehaviorAction, nav2_core::Behavior)
