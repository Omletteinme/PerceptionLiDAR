"""Mission: detect blockages, plan an uncertainty-aware route, and drive it. Start robot_sim.launch.py and
localization.launch.py (filter:=both or filter:=reliability, so the /reliability/* topics exist) first.

  use_reliability:=false   blockage detector counts every valid beam (ablation; the default gates by per-beam reliability)
  goal:=J4                 goal node (default: the goal in network_graph.yaml)
  follow:=false            plan and detect only, do not drive (the follower is not started)
  w_unc:=2.0               weight of the uncertainty term in the route cost
  system:=baseline         the whole stack (detector, planner, follower) runs on the BASELINE filter's pose, with no scan
                           reliability anywhere (ungated detector, constant R in the planner); default: reliability-aware
  network:=sewer_network_v2  use the v2 map and graph (default sewer_network)
  health_response:=false   the follower ignores the localization health (default: slows when DEGRADED, waits when CRITICAL)

All nodes are CUSTOM (sewer_perception, sewer_planning). The follower drives on the ESTIMATED pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context):
    from launch.substitutions import LaunchConfiguration as LC
    cfg = {k: LC(k).perform(context) for k in ('use_reliability', 'goal', 'follow', 'w_unc', 'health_response', 'system', 'network')}
    gz = get_package_share_directory('sewer_gazebo')
    sfx = cfg['network'][len('sewer_network'):]
    map_path = os.path.join(gz, 'maps', 'sewer_map' + sfx)
    graph_path = os.path.join(gz, 'config', f'network_graph{sfx}.yaml')
    sim = {'use_sim_time': True}
    baseline = cfg['system'].lower() == 'baseline'
    ns = '/baseline' if baseline else '/reliability'
    pose = {'pose_topic': ns + '/estimated_pose'}
    actions = [
        Node(package='sewer_perception', executable='blockage_detector_node', output='screen',
             parameters=[{'map_path': map_path, 'graph_path': graph_path,
                          'use_reliability': cfg['use_reliability'].lower() == 'true' and not baseline,
                          **pose, **sim}]),
        Node(package='sewer_planning', executable='route_planner_node', output='screen',
             parameters=[{'map_path': map_path, 'graph_path': graph_path, 'goal': cfg['goal'],
                          'w_unc': float(cfg['w_unc']), **pose,
                          **({'r_topic': '/baseline/no_reliability'} if baseline else {}), **sim}]),
    ]
    if cfg['follow'].lower() == 'true':
        actions.append(Node(package='sewer_planning', executable='route_follower_node', output='screen',
                            parameters=[{'health_topic': ns + '/localization_health' if cfg['health_response'].lower() == 'true' else '',
                                         **pose, **sim}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_reliability', default_value='true'),
        DeclareLaunchArgument('goal', default_value=''),
        DeclareLaunchArgument('follow', default_value='true'),
        DeclareLaunchArgument('w_unc', default_value='2.0'),
        DeclareLaunchArgument('health_response', default_value='true'),
        DeclareLaunchArgument('system', default_value='reliability'),
        DeclareLaunchArgument('network', default_value='sewer_network'),
        OpaqueFunction(function=_setup),
    ])
