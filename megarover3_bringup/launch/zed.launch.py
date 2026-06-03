import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    depth_mode_arg = DeclareLaunchArgument(
        'depth_mode',
        default_value='NEURAL_PLUS',
        description='ZED depth mode: NEURAL_LIGHT, NEURAL, NEURAL_PLUS',
        choices=['NEURAL_LIGHT', 'NEURAL', 'NEURAL_PLUS'],
    )
    od_arg = DeclareLaunchArgument(
        'od',
        default_value='false',
        description='Enable ZED SDK Object Detection',
        choices=['true', 'false'],
    )

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
            'param_overrides': PythonExpression([
                '"depth.depth_mode:=', LaunchConfiguration('depth_mode'),
                ';object_detection.od_enabled:=', LaunchConfiguration('od'), '"'
            ]),
        }.items(),
    )

    return LaunchDescription([depth_mode_arg, od_arg, zed_launch])
