"""The integrated system in ONE launch (build plan Phase 6): simulator + localization + Account 2 + Account 1.

  ros2 launch sewer_bringup full_mission.launch.py                       # headless, clean
  ros2 launch sewer_bringup full_mission.launch.py headless:=false       # with the Gazebo window (demo)
  ros2 launch sewer_bringup full_mission.launch.py disturbed:=true severity:=2.0 odom_disturbed:=true odom_severity:=2.0

Starts, in this order (same sequence and waits as tests/account2_live_check.sh, which was run on the laptop):
  t = 0 s    sewer_manipulation/inspection_sim.launch.py   Gazebo inspection world + inspection robot (arm, gripper,
                                                            depth camera) + bridges + collision counter   [SIMULATED]
  t = 35 s   sewer_bringup/localization.launch.py           both particle filters on the v2 map            [CUSTOM]
  t = 47 s   sewer_manipulation/inspection.launch.py        perception, inspection, arm, LiDAR self-filter [CUSTOM, A2]
  t = 57 s   sewer_bringup/mission_manager.launch.py        blockage detector, Mission Manager, follower    [CUSTOM, A1]
The waits give Gazebo time to spawn the robot and the localizers time to converge before anything moves. The Mission
Manager additionally waits in INITIALIZE for the manipulator's IDLE status (init_timeout).

Existing launch files are used unchanged; nothing here replaces robot_sim.launch.py or mission.launch.py. Lead
decisions that stay OPEN and are NOT made here: whether the localizers read /scan_self_filtered (they read /scan as
before), and moving the disposal tray outside the entrance (it stays just inside, as in Account 2's world).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC


def _inc(pkg, name, args):
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())


def generate_launch_description():
    net = 'sewer_network_v2'                        # the inspection world is built on the v2 network
    args = [
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world', default_value='sewer_inspection.sdf'),
        DeclareLaunchArgument('disturbed', default_value='false'),
        DeclareLaunchArgument('severity', default_value='1.0'),
        DeclareLaunchArgument('odom_disturbed', default_value='false'),
        DeclareLaunchArgument('odom_severity', default_value='1.0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('require_manipulator', default_value='true'),
        DeclareLaunchArgument('init_timeout', default_value='30.0'),
        DeclareLaunchArgument('t_localization', default_value='35.0'),
        DeclareLaunchArgument('t_account2', default_value='47.0'),
        DeclareLaunchArgument('t_mission', default_value='57.0'),
    ]
    sim = _inc('sewer_manipulation', 'inspection_sim.launch.py',
               {'headless': LC('headless'), 'world': LC('world')})
    loc = _inc('sewer_bringup', 'localization.launch.py',
               {'filter': 'both', 'network': net, 'disturbed': LC('disturbed'), 'severity': LC('severity'),
                'odom_disturbed': LC('odom_disturbed'), 'odom_severity': LC('odom_severity'), 'seed': LC('seed')})
    acc2 = _inc('sewer_manipulation', 'inspection.launch.py', {'record': LC('record')})
    mission = _inc('sewer_bringup', 'mission_manager.launch.py',
                   {'network': net, 'require_manipulator': LC('require_manipulator'),
                    'init_timeout': LC('init_timeout')})
    return LaunchDescription(args + [
        sim,
        TimerAction(period=LC('t_localization'), actions=[loc]),
        TimerAction(period=LC('t_account2'), actions=[acc2]),
        TimerAction(period=LC('t_mission'), actions=[mission]),
    ])
