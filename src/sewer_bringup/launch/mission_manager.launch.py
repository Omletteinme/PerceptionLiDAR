"""Full autonomous mission: explore, remove the best feasible debris, dispose at the entrance, resume, report.
Start robot_sim.launch.py and localization.launch.py (filter:=both or filter:=reliability) first, plus Account 2's
perception / manipulation nodes once they exist (topics in docs/mission_interfaces.md).

  network:=sewer_network_v2   map/graph (default v2)
  require_manipulator:=true   wait up to init_timeout for /manipulation/status IDLE; without it the mission continues
                              as inspection only (reported), it never fakes pickups
  init_timeout:=20.0          [s]
  follow:=false               decide only, do not drive (no follower)

mission_manager_node is the ONLY node sequencing tasks and publishing /route; route_follower_node is the only /cmd_vel
publisher. Do not start route_planner_node or exploration_node together with this launch file.
All nodes are CUSTOM (sewer_perception, sewer_planning). Everything runs on the ESTIMATED pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context):
    from launch.substitutions import LaunchConfiguration as LC
    cfg = {k: LC(k).perform(context) for k in ('network', 'require_manipulator', 'init_timeout', 'follow')}
    gz = get_package_share_directory('sewer_gazebo')
    sfx = cfg['network'][len('sewer_network'):]
    map_path = os.path.join(gz, 'maps', 'sewer_map' + sfx)
    graph_path = os.path.join(gz, 'config', f'network_graph{sfx}.yaml')
    sim = {'use_sim_time': True}
    pose = {'pose_topic': '/reliability/estimated_pose'}
    health = '/reliability/localization_health'
    actions = [
        Node(package='sewer_perception', executable='blockage_detector_node', output='screen',
             parameters=[{'map_path': map_path, 'graph_path': graph_path, 'use_reliability': True, **pose, **sim}]),
        Node(package='sewer_planning', executable='mission_manager_node', output='screen',
             parameters=[{'map_path': map_path, 'graph_path': graph_path, 'health_topic': health,
                          'require_manipulator': cfg['require_manipulator'].lower() == 'true',
                          'init_timeout': float(cfg['init_timeout']), **pose, **sim}]),
    ]
    if cfg['follow'].lower() == 'true':
        actions.append(Node(package='sewer_planning', executable='route_follower_node', output='screen',
                            parameters=[{'health_topic': health, 'rearm_on_new_goal': True, **pose, **sim}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network', default_value='sewer_network_v2'),
        DeclareLaunchArgument('require_manipulator', default_value='true'),
        DeclareLaunchArgument('init_timeout', default_value='20.0'),
        DeclareLaunchArgument('follow', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
