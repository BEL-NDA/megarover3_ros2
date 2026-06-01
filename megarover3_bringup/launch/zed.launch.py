import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'ros_params_override_path': os.path.join(
                get_package_share_directory('megarover3_bringup'),
                'config',
                'zed_megarover.yaml',
            ),
        }.items(),
    )

    # Connect zed_odom -> odom so the robot model is visible
    # when Fixed Frame = zed_odom in RViz.
    zed_odom_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_odom_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'zed_odom', 'odom'],
    )

    return LaunchDescription([zed_launch, zed_odom_to_odom])
