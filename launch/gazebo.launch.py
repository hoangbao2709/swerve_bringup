#!/usr/bin/env python3
"""Spawn the swerve model using the published Gazebo manifest when available."""
import json
import math
import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _advance_after_success(label, actions):
    """Advance a startup dependency chain only after a clean process exit."""
    def on_exit(event, _context):
        returncode = getattr(event, 'returncode', None)
        if returncode == 0:
            return actions
        reason = f'{label} failed with exit code {returncode}; dependent startup is aborted'
        return [LogInfo(msg=f'[ERROR] {reason}'), Shutdown(reason=reason)]

    return on_exit


def resolve_robot_spawn(world_path, robot_id, fallback, allow_dev_world=False):
    """Return ``(x, y, z, yaw, source)`` for a world and selected robot.

    Published worlds carry a sibling ``manifest.json``.  Legacy development
    worlds may explicitly use the fallback launch arguments instead.
    """
    manifest_path = Path(world_path).expanduser().resolve().parent / 'manifest.json'
    if not manifest_path.exists():
        if not allow_dev_world:
            raise RuntimeError(
                f'Gazebo world has no generated map manifest: {manifest_path}; '
                'publish a canonical map or explicitly set allow_dev_world:=true')
        return tuple(float(value) for value in fallback) + ('fallback',)
    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f'Unable to read published Gazebo manifest {manifest_path}: {exc}') from exc
    record = next((item for item in manifest.get('robots', []) if str(item.get('id', '')) == str(robot_id)), None)
    if record is None:
        raise RuntimeError(f'Robot {robot_id} is not present in published Gazebo manifest {manifest_path}')
    pose = record.get('pose')
    if not isinstance(pose, (list, tuple)) or len(pose) < 4:
        raise RuntimeError(f'Robot {robot_id} has an invalid pose in {manifest_path}')
    values = tuple(float(value) for value in pose[:4])
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f'Robot {robot_id} has a non-finite pose in {manifest_path}')
    return values + ('manifest',)


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_bringup')
    gazebo_share = get_package_share_directory('gazebo_ros')
    urdf_path = os.path.join(pkg_share, 'urdf', 'swerve_base.urdf')
    default_world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world')
    swerve_controller_config = os.path.join(
        pkg_share, 'config', 'swerve_controller.yaml')
    command_arbiter_config = os.path.join(
        pkg_share, 'config', 'command_arbiter.yaml')
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
                   ' use_cad_visuals:=', LaunchConfiguration('use_cad_visuals'),
                   ' caster_frictionless:=', LaunchConfiguration('caster_frictionless'),
                   ' proper_caster_test:=', LaunchConfiguration('proper_caster_test'),
                   ' caster_axle_offset_x_m:=', LaunchConfiguration('caster_axle_offset_x_m'),
                   ' caster_axle_offset_y_m:=', LaunchConfiguration('caster_axle_offset_y_m'),
                   ' proper_caster_mu1:=', LaunchConfiguration('proper_caster_mu1'),
                   ' proper_caster_mu2:=', LaunchConfiguration('proper_caster_mu2'),
                   ' caster_swivel_friction:=', LaunchConfiguration('caster_swivel_friction'),
                   ' caster_swivel_damping:=', LaunchConfiguration('caster_swivel_damping'),
                   ' caster_roll_friction:=', LaunchConfiguration('caster_roll_friction'),
                   ' caster_roll_damping:=', LaunchConfiguration('caster_roll_damping'),
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
            # Gazebo Classic can take over a minute to load the published
            # warehouse world on a VMware guest before gazebo_ros_factory
            # advertises /spawn_entity. Keep spawn alive through that load.
            '-timeout', '180',
            '-x', LaunchConfiguration('resolved_spawn_x'),
            '-y', LaunchConfiguration('resolved_spawn_y'),
            '-z', LaunchConfiguration('resolved_spawn_z'),
            '-R', '0.0', '-P', '0.0', '-Y', LaunchConfiguration('resolved_spawn_yaw'),
        ],
    )

    def configure_spawn(context):
        world = LaunchConfiguration('world').perform(context)
        robot_id = LaunchConfiguration('robot_id').perform(context).strip() or 'R01'
        allow_dev_world = LaunchConfiguration('allow_dev_world').perform(context).lower() == 'true'
        fallback = (
            LaunchConfiguration('spawn_x').perform(context),
            LaunchConfiguration('spawn_y').perform(context),
            LaunchConfiguration('spawn_z').perform(context),
            LaunchConfiguration('spawn_yaw').perform(context),
        )
        x, y, z, yaw, source = resolve_robot_spawn(world, robot_id, fallback, allow_dev_world)
        # Set substitutions before the concrete Node executes.  Keeping Node
        # concrete preserves the existing controller OnProcessExit chain.
        resolved = [
            SetLaunchConfiguration('resolved_spawn_x', str(x)),
            SetLaunchConfiguration('resolved_spawn_y', str(y)),
            SetLaunchConfiguration('resolved_spawn_z', str(z)),
            SetLaunchConfiguration('resolved_spawn_yaw', str(yaw)),
        ]
        if source == 'manifest':
            return resolved + [LogInfo(msg=f'Spawn {robot_id} loaded from published Gazebo manifest: x={x}, y={y}, z={z}, yaw={yaw}')]
        return resolved + [LogInfo(msg=f'Generated manifest unavailable; using development spawn fallback for {robot_id}: x={x}, y={y}, z={z}, yaw={yaw}')]

    spawn_config = OpaqueFunction(function=configure_spawn)

    # Gazebo creates /controller_manager from the gazebo_ros2_control plugin
    # while the entity is being inserted. Start controllers only after spawn
    # has completed so controller_manager is available.
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_joint_state_broadcaster',
            output='screen',
            # Keep to the controller_manager Humble CLI contract.  The
            # service/switch timeout flags were added in newer releases and
            # make the spawner exit immediately on the supported Ubuntu 22.04
            # + ROS 2 Humble package set.
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '180'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_steering_controller',
            output='screen',
            arguments=['steering_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '180'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_drive_controller',
            output='screen',
            arguments=['drive_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '180'],
        ),
    ]

    swerve_controller = Node(
        package='swerve_bringup',
        executable='swerve_controller_node',
        name='swerve_controller',
        output='screen',
        parameters=[swerve_controller_config, {'use_sim_time': use_sim_time}],
    )

    command_arbiter = Node(
        package='swerve_bringup',
        executable='command_arbiter_node',
        name='command_arbiter',
        output='screen',
        parameters=[command_arbiter_config, {'use_sim_time': use_sim_time}],
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

    # Load controllers serially, and do not advance after a failed spawner.
    # gazebo_ros2_control creates controller_manager during entity insertion;
    # each spawner returns success only after its controller is active.
    start_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=spawn,
                      on_exit=_advance_after_success('robot entity spawn', [controller_spawners[0]])))
    start_steering = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawners[0],
                      on_exit=_advance_after_success('joint_state_broadcaster activation',
                                                     [controller_spawners[1]])))
    start_drive = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawners[1],
                      on_exit=_advance_after_success('steering_controller activation',
                                                     [controller_spawners[2]])))
    start_command_arbiter = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawners[2],
                      on_exit=_advance_after_success('drive_controller activation',
                                                     [command_arbiter])))
    start_selected_velocity_consumer = RegisterEventHandler(
        OnProcessStart(target_action=command_arbiter,
                       on_start=[swerve_controller, swerve_odometry, ekf]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock for every ROS node.',
        ),
        DeclareLaunchArgument(
            'use_cad_visuals', default_value='true',
            description='Use the optimized CAD-derived visual meshes; false selects visual-only primitives.',
        ),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start the Gazebo client window; false runs gzserver without spawning gzclient.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='Gazebo world SDF path (defaults to warehouse.world)',
        ),
        DeclareLaunchArgument(
            'robot_id', default_value='R01',
            description='Robot ID selected from the published Gazebo manifest.',
        ),
        DeclareLaunchArgument('allow_dev_world', default_value='false',
                              description='Explicitly permit a development world and fallback spawn pose.'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Development-only fallback spawn X.'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Development-only fallback spawn Y.'),
        DeclareLaunchArgument('spawn_z', default_value='0.002', description='Development-only fallback spawn Z.'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Development-only fallback spawn yaw.'),
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
            'caster_axle_offset_x_m', default_value='0.0',
            description='Test-only caster wheel-axle offset from swivel axis in caster local +X (m).',
        ),
        DeclareLaunchArgument(
            'caster_axle_offset_y_m', default_value='0.0',
            description='Test-only caster wheel-axle offset from swivel axis in caster local +Y (m).',
        ),
        DeclareLaunchArgument('proper_caster_mu1', default_value='0.01', description='TEST-ONLY proper-caster isotropic contact mu1.'),
        DeclareLaunchArgument('proper_caster_mu2', default_value='0.01', description='TEST-ONLY proper-caster isotropic contact mu2.'),
        DeclareLaunchArgument('caster_swivel_friction', default_value='0.02', description='TEST-ONLY passive caster swivel joint friction.'),
        DeclareLaunchArgument('caster_swivel_damping', default_value='0.02', description='TEST-ONLY passive caster swivel joint damping.'),
        DeclareLaunchArgument('caster_roll_friction', default_value='0.02', description='TEST-ONLY passive caster roll joint friction.'),
        DeclareLaunchArgument('caster_roll_damping', default_value='0.02', description='TEST-ONLY passive caster roll joint damping.'),
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
        spawn_config,
        spawn,
        start_joint_state,
        start_steering,
        start_drive,
        start_command_arbiter,
        start_selected_velocity_consumer,
    ])
