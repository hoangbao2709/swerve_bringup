#!/usr/bin/env python3
"""Holonomic Nav2 bringup for the swerve base.

SLAM Toolbox must already be running and publishing map -> odom. This launch
therefore starts Nav2 navigation servers, not AMCL or a second map server.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_bringup')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params = os.path.join(pkg_share, 'swerve_navigation', 'config', 'nav2_params.yaml')

    common = [params_file, {'use_sim_time': use_sim_time}]
    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=common),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=common),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server', 'planner_server', 'behavior_server',
                    'bt_navigator', 'waypoint_follower',
                ],
            }],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=params,
            description='Nav2 parameter file'),
        *nodes,
    ])
