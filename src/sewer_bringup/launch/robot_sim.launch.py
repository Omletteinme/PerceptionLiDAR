"""Gazebo world + robot + ros_gz bridge + robot_state_publisher.

Launches only SIMULATION infrastructure (Gazebo, ros_gz, robot_state_publisher).
Localization / perception / planning nodes are added by later launch files.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    gz_share = get_package_share_directory('sewer_gazebo')
    desc_share = get_package_share_directory('sewer_description')

    robot_xml = xacro.process_file(os.path.join(desc_share, 'urdf', 'sewer_robot.urdf.xacro')).toxml()

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    world_path = PathJoinSubstitution([gz_share, 'worlds', world])

    # Contact sensors: Gazebo publishes one topic per sensor, named with the world; bridge them to contacts/<name>.
    gz_world = PythonExpression(["'/world/test_arena' if '", world, "'.startswith('test_arena') else '/world/sewer_network'"])
    contact_topics, contact_remaps = [], []
    for name, link in (('chassis', 'base_link'), ('left_wheel', 'left_wheel_link'), ('right_wheel', 'right_wheel_link')):
        gz = f'/model/sewer_robot/link/{link}/sensor/{link}_contact/contact'
        contact_topics.append([gz_world, gz + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'])
        contact_remaps += ['-r', [gz_world, gz + ':=contacts/' + name]]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='test_arena.sdf'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('x', default_value='-2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world_path],
                       output='screen', condition=IfCondition(headless)),
        ExecuteProcess(cmd=['gz', 'sim', '-r', world_path],
                       output='screen', condition=UnlessCondition(headless)),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_xml, 'use_sim_time': True}], output='screen'),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'sewer_robot', '-topic', 'robot_description',
                        '-x', x, '-y', y, '-z', '0.01', '-Y', yaw]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='contact_bridge', output='screen',
             arguments=contact_topics + ['--ros-args'] + contact_remaps, parameters=[{'use_sim_time': True}]),

        # Evaluation only: counts wall/debris collisions from the contact sensors (publishes /collisions).
        Node(package='sewer_eval', executable='collision_counter_node', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
