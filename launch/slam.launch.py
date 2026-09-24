#!/usr/bin/env python3
"""3D LiDAR preprocessing, 2D scan projection, and SLAM Toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    input_topic = LaunchConfiguration('input_topic')
    start_slam = LaunchConfiguration('start_slam')
    map_topic = LaunchConfiguration('map_topic')
    transform_publish_period = LaunchConfiguration('transform_publish_period')
    preprocess_config = os.path.join(pkg_share, 'config', 'lidar_preprocessing.yaml')
    scan_config = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    preprocessor = Node(
        package='swerve_bringup',
        executable='lidar_preprocessor_node',
        name='lidar_preprocessor',
        output='screen',
        parameters=[preprocess_config, {'use_sim_time': use_sim_time, 'input_topic': input_topic}],
    )
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[scan_config, {'use_sim_time': use_sim_time}],
        remappings=[('cloud_in', '/lidar/points_filtered'), ('scan', '/scan')],
    )
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time,
                                  'map_name': map_topic,
                                  'transform_publish_period': transform_publish_period}],
        condition=IfCondition(start_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock'),
        DeclareLaunchArgument('input_topic', default_value='/lidar/points',
                              description='Raw 3D cloud topic, normalized for sim or real driver'),
        DeclareLaunchArgument('start_slam', default_value='true',
                              description='Start SLAM map->odom owner; disable when V30E EKF owns map->odom'),
        DeclareLaunchArgument('map_topic', default_value='/map',
                              description='SLAM occupancy topic; simulation separates it from canonical /map.'),
        DeclareLaunchArgument('transform_publish_period', default_value='0.02',
                              description='SLAM map->odom TF period; zero when the canonical tag localizer owns TF.'),
        preprocessor,
        cloud_to_scan,
        slam,
    ])
