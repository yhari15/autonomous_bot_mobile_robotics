from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'must3r_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hariprasad',
    maintainer_email='hariprasad@todo.todo',
    description='MUSt3R 3D Reconstruction for Mobile Robot Navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'must3r_simple_node = must3r_nav.must3r_simple_node:main',
            'must3r_bridge = must3r_nav.must3r_bridge_node:main',
            'obstacle_detector = must3r_nav.obstacle_detector_node:main',
            'vision_navigator = must3r_nav.vision_navigator_node:main',
            'demo_pointcloud = must3r_nav.demo_pointcloud_node:main',
            'odom_to_tf = must3r_nav.odom_to_tf_node:main',
            'waypoint_navigator = must3r_nav.waypoint_navigator_node:main',
            'arm_trajectory_bridge = must3r_nav.arm_trajectory_bridge:main',
            'temp_map_publisher = must3r_nav.temp_map_publisher:main',
            'stereo_depth = must3r_nav.stereo_depth_node:main',
        ],
    },
)
