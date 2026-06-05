from setuptools import find_packages, setup

package_name = 'megarover_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsujita',
    maintainer_email='t.tsujita@ieee.org',
    description='Common perception nodes for Megarover real and simulated robots.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'nav2_obstacle_cloud_filter = megarover_perception.nav2_obstacle_cloud_filter:main',
            'person_tracks_to_markers = megarover_perception.person_tracks_to_markers:main',
            'zed_person_tracks = megarover_perception.zed_person_tracks:main',
        ],
    },
)
