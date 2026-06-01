import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ZED mount height above wheel axis: 1.13m
# Wheel axis height from base_link: 0.07m
# => base_link to zed_camera_link z = 1.20m
ZED_MOUNT_Z = 1.20


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

    # Connect odom -> zed_odom with camera height offset so that
    # the point cloud appears at the correct height in odom frame.
    odom_to_zed_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_zed_odom',
        arguments=['0', '0', str(ZED_MOUNT_Z), '0', '0', '0', 'odom', 'zed_odom'],
    )

    return LaunchDescription([zed_launch, odom_to_zed_odom])
