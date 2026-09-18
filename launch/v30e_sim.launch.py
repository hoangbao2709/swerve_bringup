#!/usr/bin/env python3
"""V30E simulation contract and map->odom correction filter."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('swerve_bringup')
    enabled = LaunchConfiguration('enable_v30e_sim')
    map_file = os.path.join(pkg, 'config', 'datamatrix_map.yaml')
    sim_cfg = os.path.join(pkg, 'config', 'v30e_sim.yaml')
    ekf_cfg = os.path.join(pkg, 'config', 'v30e_localization_ekf.yaml')
    tag_nav_cfg = os.path.join(pkg, 'config', 'tag_navigation.yaml')
    # Published PART 8 artifacts can be supplied at launch time.  Keeping the
    # package files as defaults preserves the development/sample workflow while
    # allowing a published revision to be the runtime source of truth.
    marker_map = LaunchConfiguration('datamatrix_map_file')
    tag_graph = LaunchConfiguration('tag_graph_file')
    reader = Node(package='swerve_bringup', executable='v30e_sim_node', name='v30e_sim_node',
                  output='screen', condition=IfCondition(enabled),
                  parameters=[sim_cfg, {'marker_map': marker_map,
                                        'use_sim_time': LaunchConfiguration('use_sim_time')}])
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_v30e',
               output='screen', condition=IfCondition(enabled),
               parameters=[ekf_cfg, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
               remappings=[('odometry/filtered', '/odometry/v30e')])
    tag_navigation = Node(package='swerve_bringup', executable='tag_route_planner',
                          name='tag_route_planner', output='screen', condition=IfCondition(enabled),
                          parameters=[tag_nav_cfg, {'tag_graph_file': tag_graph,
                                                    'use_sim_time': LaunchConfiguration('use_sim_time')}])
    return LaunchDescription([
        DeclareLaunchArgument('enable_v30e_sim', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('datamatrix_map_file', default_value=map_file),
        DeclareLaunchArgument('tag_graph_file', default_value=os.path.join(pkg, 'config', 'tag_graph.yaml')),
        reader, tag_navigation, ekf])
