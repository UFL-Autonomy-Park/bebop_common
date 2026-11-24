# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Starts the single-agent Bebop 2 stack: mocap, driver, teleop, filter, safety/low-level
controller and visualization. Off-board code (e.g. bebop_guidance) runs on top of this.

Required argument: params_file (YAML with every node's parameters; nodes have no defaults).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bebop_num_str = LaunchConfiguration('bebop_number').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    save_data_val = LaunchConfiguration('save_data').perform(context).lower()
    bag_name_str = LaunchConfiguration('bag_name').perform(context)
    extra_bag_topics = LaunchConfiguration('extra_bag_topics').perform(context)

    namespace = f'bebop{bebop_num_str}'
    bebop_ip = f'192.168.1.{bebop_num_str}'

    nodes = [
        Node(
            package='vrpn_mocap', executable='client_node', name='vrpn_mocap_client',
            namespace='vrpn_mocap', parameters=[params_file], output='screen'),
        Node(
            package='ros2_bebop_driver', executable='bebop_driver', name='bebop_driver',
            namespace=namespace, parameters=[params_file, {'bebop_ip': bebop_ip}], output='screen'),
        Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[params_file], output='screen'),
        Node(
            package='bebop_teleop', executable='bebop_teleop_node', name='bebop_teleop',
            namespace=namespace, parameters=[params_file], output='screen'),
        Node(
            package='rho_filter', executable='rho_filter_node', name='rho_filter',
            namespace=namespace, output='screen',
            parameters=[params_file, {
                'frames.parent': 'odom',
                'frames.child': f'{namespace}/base_link',
                'topics.input_pose': f'/vrpn_mocap/{namespace}/pose',
            }]),
        Node(
            package='bebop_control', executable='bebop_control_node', name='bebop_control',
            namespace=namespace, parameters=[params_file], output='screen'),
        Node(
            package='indoor_lab_viz', executable='indoor_lab_viz_node', name='indoor_lab_viz',
            namespace='indoor_lab_viz', parameters=[params_file], output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='world_to_odom_tf',
            arguments=['--frame-id', 'world', '--child-frame-id', 'odom'], output='screen'),
    ]

    if rviz_config:
        # The RViz file may contain BEBOP_NAMESPACE, which is replaced with the active namespace.
        with open(rviz_config, 'r') as f:
            rendered = f.read().replace('BEBOP_NAMESPACE', namespace)
        rendered_path = f'/tmp/bebop_view_{namespace}.rviz'
        with open(rendered_path, 'w') as f:
            f.write(rendered)
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rendered_path]))

    if save_data_val == 'true':
        topics = [
            f'/vrpn_mocap/{namespace}/pose',
            f'/{namespace}/filtered_odom',
            f'/{namespace}/cmd_vel',
            f'/{namespace}/mode',
        ] + extra_bag_topics.replace('{ns}', namespace).split()
        bag_cmd = (
            f"ros2 bag record -s mcap {' '.join(topics)} "
            f"-o ~/Desktop/{bag_name_str}_$(date +%Y_%m_%d-%H_%M_%S)"
        )
        nodes.append(ExecuteProcess(cmd=['bash', '-c', bag_cmd], output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='Absolute path to the YAML parameter file (required)'),
        DeclareLaunchArgument('bebop_number', default_value='106', description='Drone number; sets namespace bebop<N> and IP 192.168.1.<N>'),
        DeclareLaunchArgument('rviz_config', default_value='', description='Path to an RViz config; RViz is skipped if empty'),
        DeclareLaunchArgument('save_data', default_value='true', description='Record a rosbag to ~/Desktop'),
        DeclareLaunchArgument('bag_name', default_value='bebop_data', description='Rosbag name prefix'),
        DeclareLaunchArgument('extra_bag_topics', default_value='', description='Space-separated extra topics to record; {ns} expands to the namespace'),
        OpaqueFunction(function=launch_setup),
    ])
