import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
        }.items(),
    )

    return LaunchDescription([zed_launch])
