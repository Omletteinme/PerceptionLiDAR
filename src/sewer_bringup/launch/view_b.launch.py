"""View B: localization and navigation in RViz2. Start robot_sim.launch.py and localization.launch.py first.

Shows the prior map (map_server), the LiDAR as measured and as used by the filter (coloured by per-beam
reliability), both particle clouds, both estimates with 1-sigma ellipses, status text, and the TRUE pose.
The true pose comes from /ground_truth/odom (frame `world`, tied to `map` by an identity static transform
below). That is for visualization and evaluation only: the localizers never read it.

  rviz:=false   start only the map server and the static transform (e.g. headless checks)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = os.path.join(get_package_share_directory('sewer_gazebo'), 'maps', 'sewer_map.yaml')
    rviz_cfg = os.path.join(get_package_share_directory('sewer_bringup'), 'config', 'view_b.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        # Nav2 infrastructure: serves the a-priori map on /map
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server'],
                                           'use_sim_time': True}]),
        # The robot is spawned at the world origin, so map == world (visualization of ground truth only)
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_world',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'map', '--child-frame-id', 'world']),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
