# Copyright 2026 Alessio Morale <alessiomorale-at-gmail.com>
# SPDX-License-Identifier: mit

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("ism330dhcx_node"), "config", "ism330dhcx.yaml"
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="ism330dhcx_node",
                executable="ism330dhcx_node",
                name="ism330dhcx_node",
                parameters=[params_file],
                output="screen",
            ),
        ]
    )
