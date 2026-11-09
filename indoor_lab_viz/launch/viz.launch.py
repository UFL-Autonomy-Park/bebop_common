# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='indoor_lab_viz',
            executable='indoor_lab_viz_node',
            name='indoor_lab_viz_node',
            namespace='indoor_lab_viz',
            parameters=[os.path.join(get_package_share_directory('indoor_lab_viz'), 'param', 'lab_geometry.yaml')],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('indoor_lab_viz'), 'rviz', 'indoor_lab.rviz')]
        )
    ])