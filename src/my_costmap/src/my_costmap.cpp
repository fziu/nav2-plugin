#include "my_costmap/my_costmap.hpp"

namespace my_costmap
{
    MyCostmap::MyCostmap() : obstacle_cost_(253) {}

    void MyCostmap::onInitialize()
    {
        auto node = node_.lock();
        logger_ = node->get_logger();
    }

    void MyCostmap::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        *min_x = -1.0;
        *min_y = -1.0;
        *max_x = 1.0;
        *max_y = 1.0;
    }

    void MyCostmap::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        // RCLCPP_INFO(logger_, "fuck min_i: %d max_i: %d", min_i, max_i);
        // RCLCPP_INFO(logger_, "fuck min_j: %d max_j: %d", min_j, max_j);

        unsigned char * master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++) {
            unsigned int it = span * j + min_i;
            for (int i = min_i; i < max_i; i++) {
                master[it] = obstacle_cost_;
                it++;
            }
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_costmap::MyCostmap, nav2_costmap_2d::Layer)
