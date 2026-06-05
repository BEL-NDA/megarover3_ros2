import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('megarover3_navigation')
    internal_launch_dir = os.path.join(package_dir, 'launch', 'internal')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    start_random_goals = LaunchConfiguration('start_random_goals')
    start_obstacle_cloud_filter = LaunchConfiguration('start_obstacle_cloud_filter')
    show_costmap_cloud_ranges = LaunchConfiguration('show_costmap_cloud_ranges')

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(internal_launch_dir, 'nav2_stack.launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': container_name,
            'log_level': log_level,
        }.items(),
    )

    random_goal_sender = Node(
        condition=IfCondition(start_random_goals),
        package='megarover3_navigation',
        executable='random_goal_sender.py',
        name='random_goal_sender',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': LaunchConfiguration('goal_frame'),
            'x_min': LaunchConfiguration('goal_x_min'),
            'x_max': LaunchConfiguration('goal_x_max'),
            'y_min': LaunchConfiguration('goal_y_min'),
            'y_max': LaunchConfiguration('goal_y_max'),
            'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
            'goal_pause_sec': LaunchConfiguration('goal_pause_sec'),
            'initial_delay_sec': LaunchConfiguration('initial_delay_sec'),
            'random_yaw': LaunchConfiguration('random_yaw'),
            'seed': LaunchConfiguration('seed'),
        }],
    )

    obstacle_cloud_filter = Node(
        condition=IfCondition(start_obstacle_cloud_filter),
        package='megarover_perception',
        executable='nav2_obstacle_cloud_filter',
        name='nav2_obstacle_cloud_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/zed/zed_node/point_cloud/cloud_registered',
            'output_topic': '/perception/nav2_obstacle_cloud',
            'target_frame': 'base_footprint',
            'min_x': 0.20,
            'max_x': 4.0,
            'min_y': -2.5,
            'max_y': 2.5,
            'min_z': 0.12,
            'max_z': 1.8,
            'ground_filter_enabled': True,
            'ground_sample_min_z': -0.20,
            'ground_sample_max_z': 0.45,
            'ground_xy_leaf_size': 0.20,
            'ground_clearance': 0.16,
            'voxel_leaf_size': 0.06,
            'radius_outlier_radius': 0.18,
            'radius_outlier_min_neighbors': 2,
            'max_publish_hz': 8.0,
        }],
    )

    costmap_cloud_range_markers = Node(
        condition=IfCondition(show_costmap_cloud_ranges),
        package='megarover3_navigation',
        executable='costmap_cloud_range_markers.py',
        name='costmap_cloud_range_markers',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'height_min': 0.15,
            'height_max': 1.8,
            'local_width': 5.0,
            'local_height': 5.0,
            'local_obstacle_min_range': 0.20,
            'local_obstacle_max_range': 3.5,
            'local_raytrace_max_range': 4.0,
            'global_width': 24.0,
            'global_height': 24.0,
            'global_obstacle_min_range': 0.20,
            'global_obstacle_max_range': 4.0,
            'global_raytrace_max_range': 5.0,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_dir, 'config', 'zed_pointcloud_odom_nav2_params.yaml'),
            description='Nav2 params file.',
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('start_random_goals', default_value='true'),
        DeclareLaunchArgument('start_obstacle_cloud_filter', default_value='true'),
        DeclareLaunchArgument('show_costmap_cloud_ranges', default_value='true'),
        DeclareLaunchArgument('goal_frame', default_value='odom'),
        DeclareLaunchArgument('goal_x_min', default_value='-5.0'),
        DeclareLaunchArgument('goal_x_max', default_value='5.0'),
        DeclareLaunchArgument('goal_y_min', default_value='-5.0'),
        DeclareLaunchArgument('goal_y_max', default_value='5.0'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='180.0'),
        DeclareLaunchArgument('goal_pause_sec', default_value='2.0'),
        DeclareLaunchArgument('initial_delay_sec', default_value='5.0'),
        DeclareLaunchArgument('random_yaw', default_value='false'),
        DeclareLaunchArgument('seed', default_value='0'),
        obstacle_cloud_filter,
        nav2_stack,
        costmap_cloud_range_markers,
        random_goal_sender,
    ])
