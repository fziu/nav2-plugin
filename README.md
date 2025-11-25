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