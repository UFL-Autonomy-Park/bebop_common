import datetime
import os
from pathlib import Path

import pytz
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def cfg_path(ros_package_name: str, subdirectory: str, filename: str):
    return os.path.join(
        get_package_share_directory(ros_package_name), subdirectory, filename
    )


NCR_PARAM = cfg_path(
    ros_package_name="bebop_bringup",
    subdirectory="param",
    filename="ncr_param.yaml",
)
CLIENT_PARAM = cfg_path(
    ros_package_name="bebop_bringup",
    subdirectory="param",
    filename="client_param.yaml",
)
SERVER_PARAM = cfg_path(
    ros_package_name="bebop_bringup",
    subdirectory="param",
    filename="server_param.yaml",
)
BEBOP_PARAM = cfg_path(
    ros_package_name="bebop_bringup",
    subdirectory="param",
    filename="bebop_param.yaml",
)
CBF_PARAM = cfg_path(
    ros_package_name="bebop_bringup",
    subdirectory="param",
    filename="cbf_param.yaml",
)


# for rosbag
SAVE_DATA = False
date = datetime.datetime.now()
tz = pytz.timezone("America/New_York")
date = tz.localize(date)
OUTPUT_DIR = (
    "/home/brandonfallin@ad.ufl.edu/Desktop/experiment_data/bebop_flight_test/"
    + str(date.year - 2000)
    + str(date.month)
    + str(date.day)
    + "/"
    + str(date.hour)
    + "_"
    + str(date.minute)
)
BAG_NAME = "bebop_flight_test"
AGENT_NAMES = ["bebop112", "bebop113"]

pose_list = [f"/vrpn_mocap/{agent_name}/pose" for agent_name in (AGENT_NAMES)]

cmd_vel_list = [
    "/bebop112/cmd_vel_des",
    "/bebop112/cmd_vel_des",
]


topic_list = pose_list + cmd_vel_list
rosbag_cmd = ["ros2", "bag", "record", *topic_list, "-o", f"{OUTPUT_DIR}/{BAG_NAME}"]


def generate_launch_description():
    launch_actions: list[Action] = [
        Node(
            package="vrpn_mocap",
            executable="client_node",
            name="vrpn_mocap_client_node",
            namespace="vrpn_mocap",
            parameters=[NCR_PARAM],
            output="screen",
        ),
        Node(
            package="indoor_lab_viz",
            executable="indoor_lab_viz_node",
            name="indoor_lab_viz_node",
            namespace="indoor_lab_viz",
            parameters=[NCR_PARAM],
            output="screen",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[NCR_PARAM],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
            output="screen",
        ),
        # multiple bebops
        Node(
            package="bebop_server",
            executable="bebop_server_node",  # check this
            name="bebop_server_node",
            parameters=[SERVER_PARAM],
            output="screen",
        ),
        # bebop112 support
        Node(
            package="ros2_bebop_driver",
            executable="bebop_driver",  # check this
            name="bebop_driver_node",
            namespace="bebop112",
            parameters=[BEBOP_PARAM],
            output="screen",
        ),
        Node(
            package="rho_filter",
            executable="rho_filter_node",
            name="rho_filter_node",
            namespace="bebop112",
            parameters=[BEBOP_PARAM],
            output="screen",
        ),
        Node(
            package="bebop_control",
            executable="bebop_control_node",
            name="bebop_control_node",
            namespace="bebop112",
            parameters=[BEBOP_PARAM],
            output="screen",
        ),
        Node(
            package="bebop_client",
            executable="bebop_client_node",
            name="bebop_client_node",
            namespace="bebop112",
            parameters=[CLIENT_PARAM],
            output="screen",
        ),
        # bebop113 support
        Node(
            package="ros2_bebop_driver",
            executable="bebop_driver",  # check this
            name="bebop_driver_node",
            namespace="bebop113",
            parameters=[BEBOP_PARAM],
            output="screen",
        ),
        Node(
            package="rho_filter",
            executable="rho_filter_node",
            name="rho_filter_node",
            namespace="bebop113",
            parameters=[BEBOP_PARAM],
            output="screen",
        ),
        Node(
            package="bebop_control",
            executable="bebop_control_node",
            name="bebop_control_node",
            namespace="bebop113",
            parameters=[BEBOP_PARAM],
            output="screen",
        ),
        Node(
            package="bebop_client",
            executable="bebop_client_node",
            name="bebop_client_node",
            namespace="bebop113",
            parameters=[CLIENT_PARAM],
            output="screen",
        ),
        Node(
            package="ncr_cbf",
            executable="ncr_cbf_node",
            name="ncr_cbf_node",
            parameters=[CBF_PARAM],
            output="screen",
        ),
    ]

    if SAVE_DATA:
        launch_actions.append(ExecuteProcess(cmd=rosbag_cmd, output="screen"))

    return LaunchDescription(launch_actions)
