#include "my_controller/my_controller.hpp"

namespace my_controller
{

void MyController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = node_.lock();

    plugin_name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    pub_ = node->create_publisher<std_msgs::msg::String>("my_test", 1);
    
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".linear_velocity", rclcpp::ParameterValue(0.8)
    );
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".angular_velocity", rclcpp::ParameterValue(1.0)
    );
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".distance_goal_tolerance", rclcpp::ParameterValue(0.05)
    );
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".head_tolerance", rclcpp::ParameterValue(0.05)
    );
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.05)
    );

    node->get_parameter(plugin_name_ + ".linear_velocity", linear_velocity);
    node->get_parameter(plugin_name_ + ".angular_velocity", angular_velocity);
    node->get_parameter(plugin_name_ + ".distance_goal_tolerance", distance_goal_tolerance);
    node->get_parameter(plugin_name_ + ".head_tolerance", head_tolerance);
    node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance);
}

void MyController::cleanup()
{
    pub_.reset();
    RCLCPP_INFO(logger_, "Cleaning up controller: %s", plugin_name_.c_str());
}

void MyController::activate()
{
    is_active_ = true;
    pub_->on_activate();
    RCLCPP_INFO(logger_, "Activating controller: %s", plugin_name_.c_str());
}

void MyController::deactivate()
{
    is_active_ = false;
    pub_->on_deactivate();
    RCLCPP_INFO(logger_, "Dectivating controller: %s", plugin_name_.c_str());
}

void MyController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
    (void) speed_limit;
    (void) percentage;
}

geometry_msgs::msg::TwistStamped MyController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
    (void) velocity;
    (void) goal_checker;

    geometry_msgs::msg::TwistStamped cmd_vel;
    std_msgs::msg::String msg;

    if (!is_active_ || global_plan_.poses.empty()) {
        return cmd_vel;
    }

    auto goal = global_plan_.poses.back().pose;
    double cur_yaw = ConvertRPY(pose.pose.orientation);
    double tar_yaw = ConvertRPY(goal.orientation);
    double distance_goal = GetDistanceGoal(pose.pose.position.x, pose.pose.position.y, goal.position.x, goal.position.y);
    double head_error = GetHeadError(pose.pose.position.x, pose.pose.position.y, cur_yaw, goal.position.x, goal.position.y);
    double yaw_goal = GetYawGoal(cur_yaw, tar_yaw);
    
    if (std::fabs(distance_goal) > distance_goal_tolerance) {
        if (std::fabs(head_error) > head_tolerance) {
            if (head_error > 0)
                cmd_vel.twist.angular.z = angular_velocity;
            else
                cmd_vel.twist.angular.z = -angular_velocity;
        } else {
            cmd_vel.twist.linear.x = linear_velocity;
        }
    } else if (std::fabs(yaw_goal) > yaw_goal_tolerance) {
        if (yaw_goal > 0)
            cmd_vel.twist.angular.z = angular_velocity;
        else
            cmd_vel.twist.angular.z = -angular_velocity;
    } else {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
    }

    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    return cmd_vel;
}

void MyController::setPlan(const nav_msgs::msg::Path &path)
{
    (void) path;
    global_plan_ = path;
}

geometry_msgs::msg::Quaternion MyController::ConvertQuaternion(double yaw)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion qua_;
    q.setRPY(0, 0, yaw);
    qua_ = tf2::toMsg(q);
    return qua_;
}

double MyController::ConvertRPY(geometry_msgs::msg::Quaternion qua_)
{
    double roll, pitch, yaw;
    tf2::Quaternion quaternion_(qua_.x, qua_.y, qua_.z, qua_.w);
    tf2::Matrix3x3(quaternion_).getRPY(roll, pitch, yaw);
    return yaw;
}

double MyController::GetDistanceGoal(double cur_x, double cur_y, double tar_x, double tar_y)
{
    return std::sqrt(std::pow(tar_x - cur_x, 2) + std::pow(tar_y - cur_y, 2));
}

double MyController::GetHeadError(double cur_x, double cur_y, double cur_yaw, double tar_x, double tar_y)
{
    double tar_head = atan2(tar_y - cur_y, tar_x - cur_x);
    double head_error = tar_head - cur_yaw;
    if (head_error > M_PI) {
        head_error -= 2 * M_PI;
    } else if (head_error < -M_PI) {
        head_error += 2 * M_PI;
    }
    return head_error;
}

double MyController::GetYawGoal(double cur_yaw, double tar_yaw)
{
    return tar_yaw - cur_yaw;
}

}

PLUGINLIB_EXPORT_CLASS(my_controller::MyController, nav2_core::Controller)
