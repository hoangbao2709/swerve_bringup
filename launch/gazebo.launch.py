#!/usr/bin/env python3
"""Spawn the audited swerve model in Gazebo Classic for Phase 1 testing."""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_bringup')
    gazebo_share = get_package_share_directory('gazebo_ros')
    urdf_path = os.path.join(pkg_share, 'urdf', 'swerve_base.urdf')
    default_world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world')
    swerve_controller_config = os.path.join(
        pkg_share, 'config', 'swerve_controller.yaml')
    swerve_odometry_config = os.path.join(
        pkg_share, 'config', 'swerve_odometry.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    interface_config_path = os.path.join(pkg_share, 'config', 'sim_real_interface.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    with open(interface_config_path, encoding='utf-8') as stream:
        interface = yaml.safe_load(stream)
        lidar = interface['lidar']
        lidar_extrinsics = interface['lidar_extrinsics']
        imu_extrinsics = interface['imu_extrinsics']
    with open(interface_config_path, encoding='utf-8') as stream:
        imu = yaml.safe_load(stream)['imu']

    def xacro_arg(name, value):
        return [f' {name}:={value}']

    xacro_args = []
    for name, value in zip(('lidar_x', 'lidar_y', 'lidar_z'), lidar_extrinsics['xyz']):
        xacro_args += xacro_arg(name, value)
    for name, value in zip(('lidar_roll', 'lidar_pitch', 'lidar_yaw'), lidar_extrinsics['rpy']):
        xacro_args += xacro_arg(name, value)
    for name, value in zip((
            'horizontal_fov', 'vertical_fov', 'horizontal_samples',
            'vertical_samples', 'min_range', 'max_range', 'update_rate',
            'noise_stddev'), (
                lidar['horizontal_fov'], lidar['vertical_fov'],
                lidar['horizontal_samples'], lidar['vertical_samples'],
                lidar['min_range'], lidar['max_range'], lidar['update_rate'],
                lidar['noise_stddev'])):
        xacro_args += xacro_arg('lidar_' + name, value)
    for name, value in zip(('imu_x', 'imu_y', 'imu_z'), imu_extrinsics['xyz']):
        xacro_args += xacro_arg(name, value)
    for name, value in zip(('imu_roll', 'imu_pitch', 'imu_yaw'), imu_extrinsics['rpy']):
        xacro_args += xacro_arg(name, value)
    for name, value in zip(('update_rate', 'gyro_noise_stddev', 'accel_noise_stddev'), (
            imu['update_rate'], imu['gyro_noise_stddev'], imu['accel_noise_stddev'])):
        xacro_args += xacro_arg('imu_' + name, value)

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path] + xacro_args), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('start_state_publisher')),
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_swerve',
        output='screen',
        arguments=[
            '-entity', 'swerve_base',
            '-topic', 'robot_description',
            # Start at the centre of the warehouse map, heading along +X.
            # The lowest drive-wheel collision is about 1.4 mm below the
            # nominal base pose.  Start just above the floor so Gazebo can
            # settle the model without an initial collision impulse.
            '-x', '0.0', '-y', '0.0', '-z', '0.003',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
    )

    # Gazebo creates /controller_manager from the gazebo_ros2_control plugin
    # while the entity is being inserted. Start controllers only after spawn
    # has completed so controller_manager is available.
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_joint_state_broadcaster',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_steering_controller',
            output='screen',
            arguments=['steering_controller', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_drive_controller',
            output='screen',
            arguments=['drive_controller', '--controller-manager', '/controller_manager'],
        ),
    ]

    swerve_controller = Node(
        package='swerve_bringup',
        executable='swerve_controller_node',
        name='swerve_controller',
        output='screen',
        parameters=[swerve_controller_config, {'use_sim_time': use_sim_time}],
    )

    swerve_odometry = Node(
        package='swerve_bringup',
        executable='swerve_odometry_node',
        name='swerve_odometry',
        output='screen',
        parameters=[swerve_odometry_config, {'use_sim_time': use_sim_time}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('start_ekf')),
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=controller_spawners + [swerve_controller, swerve_odometry, ekf],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock for every ROS node.',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo client window.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='Gazebo world SDF path (defaults to warehouse.world)',
        ),
        DeclareLaunchArgument(
            'start_ekf', default_value='true',
            description='Start robot_localization here; system.launch.py disables it and owns the common EKF.',
        ),
        DeclareLaunchArgument(
            'start_state_publisher', default_value='true',
            description='Start robot_state_publisher; disable when an outer bringup owns it.',
        ),
        gazebo,
        state_publisher,
        spawn,
        start_controllers,
    ])
