#ifndef MY_COSTMAP_HPP
#define MY_COSTMAP_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace my_costmap
{

class MyCostmap : public nav2_costmap_2d::Layer
{
public:
    MyCostmap();

    virtual void onInitialize();
    virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
    
    virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

    virtual void reset() { return; }

    virtual bool isClearable() { return false; }

private:
    rclcpp::Logger logger_ {rclcpp::get_logger("MyCostmap")};

    int obstacle_cost_; // 障碍物的成本
    bool need_recalculation_ = true;
};

}

#endif  // MY_COSTMAP_HPP