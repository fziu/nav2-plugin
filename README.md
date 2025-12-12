# 学习nav2 plugin随手
```
FollowPath:
    plugin: "my_controller::MyController"
    debug_trajectory_details: True
    transform_tolerance: 1.0
    linear_velocity: 0.2
    angular_velocity: 0.4
    distance_goal_tolerance: 0.02
    head_tolerance: 0.05
    yaw_goal_tolerance: 0.02

plugins: ["my_layer"]
    my_layer:
    plugin: "my_costmap::MyCostmap"

GridBased:
    plugin: "my_planner::MyPlanner"
    interpolation_resolution: 0.1

behavior_server:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait", "my_behavior", "my_behavior_action"]
    my_behavior:
      plugin: "my_behavior::MyBehavior"
    my_behavior_action:
      plugin: "my_behavior_action::MyBehaviorAction"

bt_navigator:
    default_nav_to_pose_bt_xml: "$(find-pkg-share my_navigator)/behavior_trees/my_behavior_tree.xml"
    plugin_lib_names:
      - ...
      - nav2_wait_action_bt_node
      - my_action_bt_node
```


my_costmap_filters:  
launch、maps复制到turtlebot3_navigation2  

```
global_costmap:   # local_costmap
  global_costmap:
    ros__parameters:
      filters: ["keepout_filter", "speed_filter"]
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/keepout_costmap_filter_info"
      speed_filter:
        plugin: "nav2_costmap_2d::SpeedFilter"
        enabled: True
        filter_info_topic: "/speed_costmap_filter_info"
        speed_limit_topic: "/speed_limit"

keepout_filter_mask_server:
  ros__parameters:
    topic_name: "/keepout_filter_mask"

keepout_costmap_filter_info_server:
  ros__parameters:
    type: 0
    filter_info_topic: "/keepout_costmap_filter_info"
    mask_topic: "/keepout_filter_mask"
    base: 0.0
    multiplier: 1.0

speed_filter_mask_server:
  ros__parameters:
    topic_name: "/speed_filter_mask"

speed_costmap_filter_info_server:
  ros__parameters:
    type: 1
    filter_info_topic: "/speed_costmap_filter_info"
    mask_topic: "/speed_filter_mask"
    base: 100.0
    multiplier: -1.0
```