#!/usr/bin/env python3
"""Holonomic Nav2 bringup for the swerve base.

Navigation owns a saved map and the V30E EKF owns ``map -> odom``.  SLAM is
therefore deliberately not started here.
"""

import os
import re

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
    map_file = LaunchConfiguration('map_file')
    params = os.path.join(pkg_share, 'swerve_navigation', 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'swerve_navigation', 'maps', 'warehouse.yaml')
    default_pgm = os.path.join(pkg_share, 'swerve_navigation', 'maps', 'warehouse.pgm')

    def validate_saved_map(yaml_path, pgm_path):
        """Fail early instead of silently navigating on the old placeholder."""
        if not os.path.isfile(yaml_path) or not os.path.isfile(pgm_path):
            raise RuntimeError(
                'Navigation requires a real SLAM map. Missing %s or %s. '
                'Run mapping and map_saver_cli first.' % (yaml_path, pgm_path))
        with open(yaml_path, encoding='utf-8') as stream:
            metadata = stream.read()
        match = re.search(r'^resolution:\s*([0-9.]+)', metadata, re.MULTILINE)
        if not match or float(match.group(1)) > 0.10:
            raise RuntimeError(
                'warehouse map is not a SLAM navigation map (resolution must be <= 0.10 m): %s'
                % yaml_path)
        with open(pgm_path, 'rb') as stream:
            header = stream.readline().strip()
            dimensions = stream.readline().split()
            stream.readline()
            pixels = stream.read()
        if header not in (b'P5', b'P2') or len(dimensions) != 2:
            raise RuntimeError('Invalid warehouse PGM map: %s' % pgm_path)
        width, height = (int(dimensions[0]), int(dimensions[1]))
        occupied = sum(pixel < 100 for pixel in pixels)
        # The former temporary map was exactly 60x40 at 0.5 m/pixel and only
        # contained the perimeter.  Reject it even if somebody copied it back.
        if (width, height) == (60, 40) or occupied < 500:
            raise RuntimeError(
                'warehouse map is a placeholder or has no measured obstacles: '
                '%dx%d, occupied_pixels=%d. Run mapping with LiDAR and save it.'
                % (width, height, occupied))

    validate_saved_map(default_map, default_pgm)

    def nav2_executable(package, executable):
        # A stale cartoros2 overlay on this machine contains non-executable
        # copies of several Nav2 binaries. Prefer the executable Humble
        # binary when the selected overlay file is not runnable.
        system_binary = os.path.join('/opt/ros/humble', 'lib', package, executable)
        return system_binary if os.access(system_binary, os.X_OK) else executable

    nav2_env = {
        # Prevent an ABI-incompatible cartoros2 Nav2 overlay from being loaded
        # through LD_LIBRARY_PATH while this bringup uses ROS Humble binaries.
        'AMENT_PREFIX_PATH': '/opt/ros/humble',
        'LD_LIBRARY_PATH': '/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu',
    }

    # Resolve the package config directly.  Passing an empty LaunchConfiguration
    # as a parameter-file entry makes launch_ros interpret it as the current
    # directory (and silently starts Nav2 with defaults).
    common = [params, {'use_sim_time': use_sim_time}]
    nodes = [
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             additional_env=nav2_env,
             parameters=[{'use_sim_time': use_sim_time,
                          'yaml_filename': map_file}]),
        Node(package='nav2_controller', executable=nav2_executable('nav2_controller', 'controller_server'),
             name='controller_server', output='screen', parameters=common,
             additional_env=nav2_env),
        Node(package='nav2_planner', executable=nav2_executable('nav2_planner', 'planner_server'),
             name='planner_server', output='screen', parameters=common,
             additional_env=nav2_env),
        Node(package='nav2_behaviors', executable=nav2_executable('nav2_behaviors', 'behavior_server'),
             name='behavior_server', output='screen', parameters=common,
             additional_env=nav2_env),
        Node(package='nav2_bt_navigator', executable=nav2_executable('nav2_bt_navigator', 'bt_navigator'),
             name='bt_navigator', output='screen', parameters=common,
             additional_env=nav2_env),
        Node(package='nav2_waypoint_follower', executable=nav2_executable('nav2_waypoint_follower', 'waypoint_follower'),
             name='waypoint_follower', output='screen', parameters=common,
             additional_env=nav2_env),
        Node(
            package='nav2_lifecycle_manager', executable=nav2_executable('nav2_lifecycle_manager', 'lifecycle_manager'),
            name='lifecycle_manager_navigation', output='screen', additional_env=nav2_env,
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'map_server',
                    'controller_server', 'planner_server', 'behavior_server',
                    'bt_navigator', 'waypoint_follower',
                ],
            }],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map_file', default_value=default_map,
                              description='Static warehouse map YAML'),
        DeclareLaunchArgument(
            'params_file', default_value=params,
            description='Nav2 parameter file'),
        *nodes,
    ])
