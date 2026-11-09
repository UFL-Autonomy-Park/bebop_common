# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('rho_filter'),
        'config',
        'filter_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='rho_filter',
            executable='rho_filter_node',
            name='rho_filter_node',
            output='screen',
            parameters=[config_file_path],
        )
    ])