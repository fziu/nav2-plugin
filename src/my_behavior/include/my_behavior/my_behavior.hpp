#ifndef MY_BEHAVIOR_HPP
#define MY_BEHAVIOR_HPP

#include <nav2_core/behavior.hpp>
#include <std_msgs/msg/string.hpp>

namespace my_behavior
{

class MyBehavior : public nav2_core::Behavior
{
public:
    MyBehavior() = default;
    ~MyBehavior() = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override;

    void cleanup() override;

    void activate() override;

    void deactivate() override;

private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("MyBehavior")};
    rclcpp::Clock::SharedPtr clock_;

    std::string behavior_name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
};

}

#endif  // MY_BEHAVIOR_HPP