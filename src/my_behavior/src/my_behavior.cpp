#include "my_behavior/my_behavior.hpp"

namespace my_behavior
{

void MyBehavior::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker)
{
    node_ = parent;
    auto node = node_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    behavior_name_ = name;
    tf_ = tf;

    collision_checker_ = collision_checker;

    pub_ = node->create_publisher<std_msgs::msg::String>("my_behavior", 1); 
}

void MyBehavior::cleanup()
{
    pub_.reset();
}

void MyBehavior::activate()
{
    pub_->on_activate();
    std_msgs::msg::String msg;
    msg.data = "fuck ...";
    pub_->publish(msg);
}

void MyBehavior::deactivate()
{
    pub_->on_deactivate();
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_behavior::MyBehavior, nav2_core::Behavior)
