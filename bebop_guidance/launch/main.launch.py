# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Example off-board layer: starts bebop_bringup with this package's parameters, then the guidance node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_guidance = get_package_share_directory('bebop_guidance')
    pkg_bringup = get_package_share_directory('bebop_bringup')
    params_file = os.path.join(pkg_guidance, 'config', 'main.yaml')
    rviz_config = os.path.join(pkg_guidance, 'config', 'bebop_view.rviz')

    bebop_number = LaunchConfiguration('bebop_number')
    namespace = ['bebop', bebop_number]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'rviz_config': rviz_config,
            'bebop_number': bebop_number,
            'joy_type': LaunchConfiguration('joy_type'),
            'save_data': LaunchConfiguration('save_data'),
            'bag_name': LaunchConfiguration('bag_name'),
            'extra_bag_topics': '/{ns}/tracking_error /{ns}/cmd_vel_des /{ns}/trajectory_setpoint',
        }.items(),
    )

    guidance = Node(
        package='bebop_guidance', executable='bebop_guidance_node', name='bebop_guidance',
        namespace=namespace, parameters=[params_file],
        remappings=[('pose', ['/vrpn_mocap/bebop', bebop_number, '/pose'])],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('bebop_number', default_value='106'),
        DeclareLaunchArgument('save_data', default_value='true'),
        DeclareLaunchArgument('bag_name', default_value='rise_controller_data'),
        DeclareLaunchArgument('joy_type', default_value='xbox'),
        bringup,
        guidance,
    ])
