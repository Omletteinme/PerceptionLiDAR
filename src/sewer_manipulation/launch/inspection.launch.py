"""Account 2 nodes: inspection (debris + defects), manipulation (arm/gripper) and the LiDAR self-filter.

Run after inspection_sim.launch.py and sewer_bringup localization.launch.py (filter:=both, map sewer_map_v2):
  ros2 launch sewer_manipulation inspection.launch.py
Args: record:=true also records a dataset (rgb/depth/poses) to ~/sewer_ws/logs/dataset.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz = get_package_share_directory('sewer_gazebo')
    sim = {'use_sim_time': True}
    common = dict(map_path=os.path.join(gz, 'maps', 'sewer_map_v2'),
                  graph_path=os.path.join(gz, 'config', 'network_graph_v2.yaml'),
                  station_path=os.path.join(gz, 'config', 'inspection_station.yaml'))
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('evidence_dir', default_value=os.path.expanduser('~/sewer_ws/logs/evidence')),
        Node(package='sewer_perception', executable='inspection_node', output='screen',
             parameters=[sim, common, {'evidence_dir': LaunchConfiguration('evidence_dir')}]),
        Node(package='sewer_manipulation', executable='manipulation_node', output='screen',
             parameters=[sim, {'station_path': common['station_path']}]),
        Node(package='sewer_manipulation', executable='scan_self_filter_node', output='screen', parameters=[sim]),
        Node(package='sewer_perception', executable='dataset_recorder_node', output='screen', parameters=[sim],
             condition=IfCondition(LaunchConfiguration('record'))),
    ])
