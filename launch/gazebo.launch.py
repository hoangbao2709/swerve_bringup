#!/usr/bin/env python3
"""Spawn the audited swerve model in Gazebo Classic for Phase 1 testing."""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo
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

    xacro_args = [' enable_contact_sensors:=', LaunchConfiguration('contact_diagnostics'),
                   ' caster_frictionless:=', LaunchConfiguration('caster_frictionless'),
                   ' proper_caster_test:=', LaunchConfiguration('proper_caster_test'),
                   ' enable_gazebo_ros2_control:=', LaunchConfiguration('enable_gazebo_ros2_control')]
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
    model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=os.path.join(pkg_share, 'models'))

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
            # Start at the centre of the warehouse map.  The lowest CAD
            # collision is 1.4 mm below the floor at z=0; 2 mm is the
            # geometry-derived contact clearance, not a dynamics fudge.
            '-x', '0.0', '-y', '0.0', '-z', '0.002',
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
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60', '--service-call-timeout', '30', '--switch-timeout', '30'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_steering_controller',
            output='screen',
            arguments=['steering_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60', '--service-call-timeout', '30', '--switch-timeout', '30'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_drive_controller',
            output='screen',
            arguments=['drive_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60', '--service-call-timeout', '30', '--switch-timeout', '30'],
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

    # Load controllers serially.  gazebo_ros2_control only creates the
    # controller manager while the spawned entity is being inserted, so a
    # parallel spawner race leaves /joint_states and /odom silent.
    start_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[controller_spawners[0]]))
    start_steering = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawners[0], on_exit=[controller_spawners[1]]))
    start_drive = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawners[1], on_exit=[controller_spawners[2]]))
    start_nodes = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawners[2],
                      on_exit=[swerve_controller, swerve_odometry, ekf]))

    return LaunchDescription([
        LogInfo(msg=['CONTACT_DIAGNOSTICS=', LaunchConfiguration('contact_diagnostics')]),
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
            'contact_diagnostics', default_value='false',
            description='Enable temporary Gazebo contact sensors for load auditing.',
        ),
        DeclareLaunchArgument(
            'caster_frictionless', default_value='false',
            description='Test-only caster friction A/B variant; production default is false.',
        ),
        DeclareLaunchArgument(
            'proper_caster_test', default_value='false',
            description='Test-only four-caster swivel+roll model; production default is false.',
        ),
        DeclareLaunchArgument(
            'enable_gazebo_ros2_control', default_value='true',
            description='Diagnostic switch to isolate Gazebo model physics from ros2_control.',
        ),
        DeclareLaunchArgument(
            'start_state_publisher', default_value='true',
            description='Start robot_state_publisher; disable when an outer bringup owns it.',
        ),
        model_path,
        gazebo,
        state_publisher,
        spawn,
        start_joint_state,
        start_steering,
        start_drive,
        start_nodes,
    ])
