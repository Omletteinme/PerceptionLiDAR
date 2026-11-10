"""Inspection simulation (Account 2): inspection world + inspection robot (arm, gripper, depth camera) + bridges.

Same structure as sewer_bringup/robot_sim.launch.py (left unchanged), but with
  robot  sewer_description/urdf/sewer_robot_inspection.urdf.xacro
  world  sewer_gazebo/worlds/sewer_inspection.sdf (default)
  extra  bridges from config/bridge_inspection.yaml and finger contact sensors -> /contacts/{left,right}_finger
Then run the usual localization.launch.py (filter:=both, map sewer_map_v2) and this package's inspection.launch.py.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gz_share = get_package_share_directory('sewer_gazebo')
    desc_share = get_package_share_directory('sewer_description')
    me = get_package_share_directory('sewer_manipulation')
    robot_xml = xacro.process_file(os.path.join(desc_share, 'urdf', 'sewer_robot_inspection.urdf.xacro')).toxml()

    world_path = PathJoinSubstitution([gz_share, 'worlds', LaunchConfiguration('world')])
    headless = LaunchConfiguration('headless')

    gz_world = '/world/sewer_network'          # every generated sewer world is called sewer_network inside
    contact_args, remaps = [], []
    for name, link in (('chassis', 'base_link'), ('left_wheel', 'left_wheel_link'), ('right_wheel', 'right_wheel_link'),
                       ('left_finger', 'left_finger_link'), ('right_finger', 'right_finger_link')):
        gz = f'{gz_world}/model/sewer_robot/link/{link}/sensor/{link}_contact/contact'
        contact_args.append(gz + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts')
        remaps += ['-r', f'{gz}:=contacts/{name}']

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sewer_inspection.sdf'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world_path], output='screen',
                       condition=IfCondition(headless)),
        ExecuteProcess(cmd=['gz', 'sim', '-r', world_path], output='screen', condition=UnlessCondition(headless)),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_xml, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'sewer_robot', '-topic', 'robot_description', '-x', LaunchConfiguration('x'),
                        '-y', LaunchConfiguration('y'), '-z', '0.01', '-Y', LaunchConfiguration('yaw')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='base_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml'), 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='inspection_bridge', output='screen',
             parameters=[{'config_file': os.path.join(me, 'config', 'bridge_inspection.yaml'), 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='contact_bridge', output='screen',
             arguments=contact_args + ['--ros-args'] + remaps, parameters=[{'use_sim_time': True}]),

        # Evaluation only (unchanged node from sewer_eval): wall/debris collisions of chassis and wheels.
        Node(package='sewer_eval', executable='collision_counter_node', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
