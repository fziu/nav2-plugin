#ifndef MY_CONTROLLER_HPP
#define MY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

using nav2_util::declare_parameter_if_not_declared;

namespace my_controller
{

class MyController : public nav2_core::Controller
{
public:
    MyController() = default;
    ~MyController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setPlan(const nav_msgs::msg::Path & path) override;
    
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;
        
    void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

protected:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string plugin_name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_ {rclcpp::get_logger("MyController")};
    rclcpp::Clock::SharedPtr clock_;

    bool is_active_ = false;

    nav_msgs::msg::Path global_plan_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

    double linear_velocity, angular_velocity;
    double distance_goal_tolerance, head_tolerance, yaw_goal_tolerance;

    geometry_msgs::msg::Quaternion ConvertQuaternion(double yaw);
    double ConvertRPY(geometry_msgs::msg::Quaternion qua_);

    double GetDistanceGoal(double cur_x, double cur_y, double tar_x, double tar_y);
    double GetHeadError(double cur_x, double cur_y, double cur_yaw, double tar_x, double tar_y);
    double GetYawGoal(double cur_yaw, double tar_yaw);
}; 

}

#endif  // MY_CONTROLLER_HPP