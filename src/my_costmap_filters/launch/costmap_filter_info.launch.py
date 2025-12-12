import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description() -> LaunchDescription:
    turtlebot3_navigation2_dir = get_package_share_directory('turtlebot3_navigation2')

    namespace = ''
    keepout_mask_yaml_file = os.path.join(turtlebot3_navigation2_dir, 'map', 'keepout_mask.yaml')
    speed_mask_yaml_file = os.path.join(turtlebot3_navigation2_dir, 'map', 'speed_mask.yaml')
    use_sim_time = True
    autostart = True
    params_file = os.path.join(turtlebot3_navigation2_dir, 'param', 'humble', 'burger.yaml')
    container_name = 'nav2_container'
    container_name_full = (namespace, '/', container_name)

    lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server', 'speed_filter_mask_server', 'speed_costmap_filter_info_server']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    load_composable_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='keepout_filter_mask_server',
                        parameters=[configured_params, {'yaml_filename': keepout_mask_yaml_file}],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='keepout_costmap_filter_info_server',
                        parameters=[configured_params],
                        remappings=remappings
                    )
                ]
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='speed_filter_mask_server',
                        parameters=[configured_params, {'yaml_filename': speed_mask_yaml_file}],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='speed_costmap_filter_info_server',
                        parameters=[configured_params],
                        remappings=remappings
                    )
                ]
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_costmap_filters',
                        parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}]
                    )
                ]
            )        
        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(load_composable_nodes)

    return ld