"""Localizers (and optional scan disturbances) for the sewer robot. Start robot_sim.launch.py first.

  filter:=both         baseline in /baseline, reliability-aware in /reliability, same input data.
                       Only the reliability-aware one publishes map->odom (never two at once).
  filter:=baseline | reliability   a single localizer with plain topic names (/estimated_pose, ...).
  disturbed:=true      insert scan_disturbance_node and feed the localizers /scan_disturbed.
  severity, seed       disturbance severity multiplier and RNG seed (repeatable).
  odom_disturbed:=true insert odom_disturbance_node (scale errors, gyro bias, wheel-slip bursts) and feed the localizers
                       /odom_disturbed and /imu_disturbed; map->odom is still composed against the true /odom.
  network              sewer_network (default) or sewer_network_v2: which prior map the filters use
  odom_severity        slip severity (0 = no slip bursts); odom_scale multiplies the per-run odometry errors.

Both localizers and the disturbance nodes are CUSTOM nodes (sewer_localization).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in ('filter', 'disturbed', 'severity', 'seed', 'n_particles', 'odom_disturbed', 'odom_severity',
                     'odom_scale', 'network')}
    map_path = os.path.join(get_package_share_directory('sewer_gazebo'), 'maps',
                            'sewer_map' + cfg['network'][len('sewer_network'):])
    disturbed = cfg['disturbed'].lower() == 'true'
    scan_topic = '/scan_disturbed' if disturbed else '/scan'
    actions = []
    if disturbed:
        actions.append(Node(package='sewer_localization', executable='scan_disturbance_node', output='screen',
                            parameters=[{'severity': float(cfg['severity']), 'seed': int(cfg['seed']),
                                         'use_sim_time': True}]))

    odom_params = {}
    if cfg['odom_disturbed'].lower() == 'true':
        actions.append(Node(package='sewer_localization', executable='odom_disturbance_node', output='screen',
                            parameters=[{'slip_severity': float(cfg['odom_severity']), 'scale': float(cfg['odom_scale']),
                                         'seed': int(cfg['seed']) + 5000, 'use_sim_time': True}]))
        odom_params = {'odom_topic': '/odom_disturbed', 'imu_topic': '/imu_disturbed', 'tf_odom_topic': '/odom'}

    def localizer(kind, ns, tf):
        return Node(package='sewer_localization', executable='localizer_node', namespace=ns, name='localizer',
                    output='screen',
                    parameters=[{'filter': kind, 'map_path': map_path, 'scan_topic': scan_topic,
                                 'publish_tf': tf, 'n_particles': int(cfg['n_particles']),
                                 'use_sim_time': True, **odom_params}])

    if cfg['filter'] == 'both':
        actions += [localizer('baseline', 'baseline', False), localizer('reliability', 'reliability', True)]
    else:
        actions.append(localizer(cfg['filter'], '', True))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('filter', default_value='both'),
        DeclareLaunchArgument('disturbed', default_value='false'),
        DeclareLaunchArgument('severity', default_value='1.0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('n_particles', default_value='500'),
        DeclareLaunchArgument('odom_disturbed', default_value='false'),
        DeclareLaunchArgument('odom_severity', default_value='1.0'),
        DeclareLaunchArgument('odom_scale', default_value='1.0'),
        DeclareLaunchArgument('network', default_value='sewer_network'),
        OpaqueFunction(function=_setup),
    ])
