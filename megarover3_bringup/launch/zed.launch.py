import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    depth_mode = LaunchConfiguration('depth_mode').perform(context)
    slam_mode = LaunchConfiguration('slam_mode').perform(context)
    area_file = LaunchConfiguration('area_file').perform(context)
    od = LaunchConfiguration('od').perform(context)

    overrides = [f'depth.depth_mode:={depth_mode}']

    if od == 'true':
        overrides.append('object_detection.od_enabled:=true')

    if slam_mode == 'mapping':
        overrides += [
            'pos_tracking.area_memory:=true',
            'pos_tracking.enable_localization_only:=false',
            'pos_tracking.save_area_memory_on_closing:=true',
        ]
        if area_file:
            overrides.append(f'pos_tracking.area_file_path:={area_file}')
        nodes.append(Node(
            package='megarover3_bringup',
            executable='map_tf_publisher.py',
            name='map_tf_publisher',
        ))
    elif slam_mode == 'localization':
        if not area_file:
            raise RuntimeError("slam_mode:=localization requires area_file to be set")
        overrides += [
            'pos_tracking.area_memory:=true',
            f'pos_tracking.area_file_path:={area_file}',
            'pos_tracking.enable_localization_only:=true',
            'pos_tracking.save_area_memory_on_closing:=false',
        ]
        nodes.append(Node(
            package='megarover3_bringup',
            executable='map_tf_publisher.py',
            name='map_tf_publisher',
        ))
    # slam_mode == 'off': area_memory stays false (yaml default)

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            )
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_urdf': 'false',
            'publish_tf': 'false',
            'ros_params_override_path': os.path.join(
                get_package_share_directory('megarover3_bringup'),
                'config',
                'zed_megarover.yaml',
            ),
            'param_overrides': ';'.join(overrides),
        }.items(),
    )

    return [zed_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_mode',
            default_value='NEURAL_PLUS',
            description='ZED depth mode: NEURAL_LIGHT, NEURAL, NEURAL_PLUS',
            choices=['NEURAL_LIGHT', 'NEURAL', 'NEURAL_PLUS'],
        ),
        DeclareLaunchArgument(
            'slam_mode',
            default_value='off',
            description='SLAM mode: off (no area memory), mapping (build & save), localization (load & relocalize)',
            choices=['off', 'mapping', 'localization'],
        ),
        DeclareLaunchArgument(
            'area_file',
            default_value='',
            description='Path to .area file. Required for localization mode. In mapping mode, saves to this path on shutdown.',
        ),
        DeclareLaunchArgument(
            'od',
            default_value='false',
            description='Enable object detection (true/false). First run optimizes AI model for your GPU (~minutes).',
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=launch_setup),
    ])
