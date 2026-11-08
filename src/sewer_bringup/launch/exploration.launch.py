"""Exploration mission: cover every accessible corridor, then return to the entrance. Start robot_sim.launch.py and
localization.launch.py (filter:=both or filter:=reliability, so the /reliability/* topics exist) first.

Same detector and follower as mission.launch.py; exploration_node takes the place of route_planner_node (it publishes
/route), so there is still exactly one node commanding motion (route_follower_node -> /cmd_vel).

  network:=sewer_network_v2  map/graph to explore (default v2, the richer network)
  follow:=false              explore decisions only, do not drive (the follower is not started)
  health_response:=true      follower slows/waits on DEGRADED/CRITICAL and coverage is not marked while CRITICAL
  return_home:=true          drive back to the entrance (graph start node) once nothing is left to explore
  w_unc:=2.0                 weight of the uncertainty term in the route cost (same meaning as in mission.launch.py)

Topics: exploration/status, coverage/status (JSON strings), coverage/markers (RViz), route, plus the detector's.
All nodes are CUSTOM (sewer_perception, sewer_planning). Everything runs on the ESTIMATED pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context):
    from launch.substitutions import LaunchConfiguration as LC
    cfg = {k: LC(k).perform(context) for k in ('network', 'follow', 'health_response', 'return_home', 'w_unc')}
    gz = get_package_share_directory('sewer_gazebo')
    sfx = cfg['network'][len('sewer_network'):]
    map_path = os.path.join(gz, 'maps', 'sewer_map' + sfx)
    graph_path = os.path.join(gz, 'config', f'network_graph{sfx}.yaml')
    sim = {'use_sim_time': True}
    pose = {'pose_topic': '/reliability/estimated_pose'}
    health = '/reliability/localization_health' if cfg['health_response'].lower() == 'true' else ''
    actions = [
        Node(package='sewer_perception', executable='blockage_detector_node', output='screen',
             parameters=[{'map_path': map_path, 'graph_path': graph_path, 'use_reliability': True, **pose, **sim}]),
        Node(package='sewer_planning', executable='exploration_node', output='screen',
             parameters=[{'map_path': map_path, 'graph_path': graph_path, 'health_topic': health,
                          'w_unc': float(cfg['w_unc']), 'return_home': cfg['return_home'].lower() == 'true',
                          **pose, **sim}]),
    ]
    if cfg['follow'].lower() == 'true':
        actions.append(Node(package='sewer_planning', executable='route_follower_node', output='screen',
                            parameters=[{'health_topic': health, 'rearm_on_new_goal': True, **pose, **sim}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network', default_value='sewer_network_v2'),
        DeclareLaunchArgument('follow', default_value='true'),
        DeclareLaunchArgument('health_response', default_value='true'),
        DeclareLaunchArgument('return_home', default_value='true'),
        DeclareLaunchArgument('w_unc', default_value='2.0'),
        OpaqueFunction(function=_setup),
    ])
