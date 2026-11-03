"""One-command demo of the whole system: Gazebo world + robot, localizers (both filters), perception, planner, follower.

  preset:=clean              no disturbance: the robot enters, sees the walls and the debris, reroutes around the blockage
  preset:=water_debris       water reflections / spray / dropouts in the LiDAR (scan severity 2) plus normal odometry error
  preset:=water_debris_slip  the same plus wheel-slip zones (odometry severity 2): the estimate has to be held by the scans
                             while the localization-health state slows and stops the robot
  network:=sewer_network | sewer_network_v2 (default v2: two blockages, two replans)
  record:=true               also record a rosbag (bag:=<path>) with everything View B (RViz) needs
  headless:=false            show the Gazebo window (WSLg) instead of running the server only

Every preset has a fixed disturbance seed (3: it reaches the goal without a collision; seed 21 at the same severity loses the
estimate near debris B2 and is kept as a documented failure, see PLAN.md), so a run repeats. Nothing here is new code: it chains robot_sim.launch.py,
localization.launch.py and mission.launch.py with the delays the verified scripts use (Gazebo needs ~30 s to load).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

PRESETS = {   # scan severity (None = no scan disturbance), odometry slip severity (None = no odometry error), seed
    'clean': dict(scan=None, odom=None, seed=3),
    'water_debris': dict(scan=2.0, odom=0.0, seed=3),
    'water_debris_slip': dict(scan=2.0, odom=2.0, seed=3),
}
BAG_TOPICS = ['/clock', '/tf', '/joint_states', '/scan', '/scan_disturbed', '/ground_truth/odom', '/cmd_vel',
              '/baseline/estimated_pose', '/reliability/estimated_pose', '/baseline/particle_cloud',
              '/reliability/particle_cloud', '/reliability/reliability_scan', '/baseline/status_marker',
              '/reliability/status_marker', '/reliability/localization_health', '/baseline/localization_health',
              '/reliability/scan_reliability_R', '/route', '/route_info', '/blockages', '/edge_gaps', '/mission_status',
              '/collisions']


def _setup(context):
    from launch.substitutions import LaunchConfiguration as LC
    cfg = {k: LC(k).perform(context) for k in ('preset', 'headless', 'record', 'bag', 'seed', 'network')}
    if cfg['preset'] not in PRESETS:
        raise RuntimeError(f"preset must be one of {sorted(PRESETS)}, not {cfg['preset']!r}")
    p = PRESETS[cfg['preset']]
    seed = int(cfg['seed']) if cfg['seed'] else p['seed']
    share = get_package_share_directory('sewer_bringup')
    inc = lambda name, args: IncludeLaunchDescription(                       # noqa: E731
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)), launch_arguments=args.items())
    loc = {'filter': 'both', 'seed': str(seed), 'network': cfg['network']}
    if p['scan'] is not None:
        loc.update(disturbed='true', severity=str(p['scan']))
    if p['odom'] is not None:
        loc.update(odom_disturbed='true', odom_severity=str(p['odom']))
    actions = [
        inc('robot_sim.launch.py', dict(headless=cfg['headless'], world=cfg['network'] + '.sdf', x='0.0', y='0.0', yaw='0.0')),
        TimerAction(period=30.0, actions=[inc('localization.launch.py', loc)]),
        TimerAction(period=40.0, actions=[inc('mission.launch.py', {'network': cfg['network']})]),
    ]
    if cfg['record'].lower() == 'true':
        actions.append(TimerAction(period=38.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', cfg['bag'], '--use-sim-time'] + BAG_TOPICS, output='screen')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('preset', default_value='water_debris_slip'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('network', default_value='sewer_network_v2'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag', default_value='demo_bag'),
        DeclareLaunchArgument('seed', default_value='', description='override the preset seed'),
        OpaqueFunction(function=_setup),
    ])
