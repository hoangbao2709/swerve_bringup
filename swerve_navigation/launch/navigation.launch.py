#!/usr/bin/env python3
"""Holonomic Nav2 bringup for the swerve base.

``REGISTERED_CANONICAL`` loads the published full warehouse map on a distinct
``/canonical_map`` topic. The ROS bridge registers that raster into the live
SLAM ``map`` frame and publishes ``/navigation_map`` for Nav2. ``LIVE_SLAM``
and ``STATIC_MAP`` remain available for their explicit legacy launch modes.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_pgm(path: Path) -> tuple[bytes, int, int, int, bytes | list[int]]:
    """Read a PGM header and pixels without adding Pillow as a dependency."""
    with path.open('rb') as stream:
        def token() -> bytes:
            value = bytearray()
            while True:
                char = stream.read(1)
                if not char:
                    raise ValueError(f'incomplete PGM header: {path}')
                if char == b'#':
                    stream.readline()
                    continue
                if char.isspace():
                    if value:
                        return bytes(value)
                    continue
                value.extend(char)

        magic = token()
        width = int(token())
        height = int(token())
        max_value = int(token())
        if magic == b'P5':
            pixels: bytes | list[int] = stream.read()
        elif magic == b'P2':
            pixels = [int(item) for item in stream.read().split()]
        else:
            raise ValueError(f'unsupported map image format {magic!r}: {path}')
    return magic, width, height, max_value, pixels


def _validate_saved_map(yaml_path: str, default_map: Path) -> Path:
    """Validate the selected map YAML and the image referenced by that YAML."""
    map_path = Path(yaml_path).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError(f'Navigation map_file does not exist: {map_path}')
    try:
        document = yaml.safe_load(map_path.read_text(encoding='utf-8')) or {}
        resolution = float(document['resolution'])
        image_name = str(document['image'])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise RuntimeError(f'Invalid Nav2 map YAML {map_path}: {exc}') from exc
    if resolution <= 0.0 or resolution > 0.25:
        raise RuntimeError(f'Nav2 map resolution must be in (0, 0.25] m: {map_path}')

    image_path = Path(image_name).expanduser()
    if not image_path.is_absolute():
        image_path = (map_path.parent / image_path).resolve()
    if not image_path.is_file():
        raise RuntimeError(f'Nav2 map image referenced by {map_path} does not exist: {image_path}')
    try:
        magic, width, height, max_value, pixels = _read_pgm(image_path)
    except (OSError, ValueError) as exc:
        raise RuntimeError(f'Invalid Nav2 map image {image_path}: {exc}') from exc
    if width <= 0 or height <= 0 or max_value <= 0:
        raise RuntimeError(f'Invalid Nav2 map dimensions/header: {image_path}')
    expected = width * height
    if len(pixels) < expected:
        raise RuntimeError(f'Nav2 map image is truncated: expected {expected} pixels, got {len(pixels)}: {image_path}')
    occupied = sum(value < 100 for value in pixels[:expected])
    # Keep the guard against the repository's known perimeter-only placeholder,
    # but do not reject a legitimate small custom map supplied by the operator.
    if map_path == default_map.resolve() and ((width, height) == (60, 40) or occupied < 500):
        raise RuntimeError(
            'the package warehouse map is a placeholder or has no measured obstacles; '
            'run mapping and map_saver_cli first'
        )
    return map_path


def _nav2_environment():
    # Keep unrelated overlays out of native Humble Nav2, but retain this
    # workspace's version-gated bondcpp repair. Shell ldd alone is insufficient:
    # additional_env also controls the actual lifecycle nodes' library loader.
    library_paths = ['/opt/ros/humble/lib', '/usr/lib/x86_64-linux-gnu']
    compat_directory = Path(get_package_prefix('swerve_bringup')) / 'lib'
    if (compat_directory / 'libbondcpp.so').is_file():
        library_paths.insert(0, str(compat_directory))
    return {
        'AMENT_PREFIX_PATH': '/opt/ros/humble',
        'LD_LIBRARY_PATH': ':'.join(library_paths),
    }


def _nav_nodes(context, *, params_default: str, default_map: Path):
    map_source = LaunchConfiguration('map_source').perform(context).strip().upper()
    localization_backend = LaunchConfiguration('localization_backend').perform(context).strip().upper()
    if map_source not in ('LIVE_SLAM', 'STATIC_MAP', 'REGISTERED_CANONICAL'):
        raise RuntimeError(
            f'unsupported Nav2 map_source={map_source!r}; choose LIVE_SLAM, STATIC_MAP, '
            'or REGISTERED_CANONICAL')
    if localization_backend not in ('AMCL', 'SET_POSE'):
        raise RuntimeError(
            f'unsupported localization_backend={localization_backend!r}; choose AMCL or SET_POSE')
    if map_source != 'STATIC_MAP' and localization_backend != 'AMCL':
        raise RuntimeError('SET_POSE localization is only supported with STATIC_MAP')
    map_path = None
    if map_source in ('STATIC_MAP', 'REGISTERED_CANONICAL'):
        map_path = _validate_saved_map(LaunchConfiguration('map_file').perform(context), default_map)
        if map_path == default_map.resolve() and LaunchConfiguration('allow_dev_map').perform(context).lower() != 'true':
            raise RuntimeError(
                'The package Nav2 map is development-only; provide the selected published revision map '
                'or explicitly set allow_dev_map:=true')
    params_path = Path(LaunchConfiguration('params_file').perform(context) or params_default).expanduser().resolve()
    if not params_path.is_file():
        raise RuntimeError(f'Nav2 params_file does not exist: {params_path}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bond_timeout = LaunchConfiguration('bond_timeout')

    def nav2_executable(package, executable):
        # Prefer the executable Humble binary when another sourced overlay has
        # installed a non-runnable or ABI-incompatible copy.
        system_binary = os.path.join('/opt/ros/humble', 'lib', package, executable)
        return system_binary if os.access(system_binary, os.X_OK) else executable

    nav2_env = _nav2_environment()
    common = [str(params_path), {'use_sim_time': use_sim_time}]
    nodes = []
    if map_source == 'STATIC_MAP':
        nodes.append(Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                          additional_env=nav2_env,
                          parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': str(map_path)}],
                          remappings=[('map', '/navigation_map')]))
        if localization_backend == 'AMCL':
            nodes.append(Node(
                package='nav2_amcl', executable=nav2_executable('nav2_amcl', 'amcl'),
                name='amcl', output='screen',
                additional_env=nav2_env,
                parameters=common,
                remappings=[('map', '/navigation_map'), ('scan', '/scan'),
                            ('initialpose', '/initialpose')],
            ))
    elif map_source == 'REGISTERED_CANONICAL':
        nodes.append(Node(package='nav2_map_server', executable='map_server', name='canonical_map_server', output='screen',
                          additional_env=nav2_env,
                          parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': str(map_path)}],
                          remappings=[('map', '/canonical_map')]))
    global_map_remapping = [('/navigation_map', '/map')] if map_source == 'LIVE_SLAM' else []
    nodes.extend([
        Node(package='nav2_controller', executable=nav2_executable('nav2_controller', 'controller_server'),
             name='controller_server', output='screen', parameters=common, additional_env=nav2_env,
             remappings=[('cmd_vel', '/cmd_vel_nav'), *global_map_remapping]),
        Node(package='nav2_planner', executable=nav2_executable('nav2_planner', 'planner_server'),
             name='planner_server', output='screen', parameters=common, additional_env=nav2_env,
             remappings=global_map_remapping),
        Node(package='nav2_behaviors', executable=nav2_executable('nav2_behaviors', 'behavior_server'),
             name='behavior_server', output='screen', parameters=common, additional_env=nav2_env,
             remappings=[('cmd_vel', '/cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable=nav2_executable('nav2_bt_navigator', 'bt_navigator'),
             name='bt_navigator', output='screen', parameters=common, additional_env=nav2_env),
        Node(package='nav2_waypoint_follower', executable=nav2_executable('nav2_waypoint_follower', 'waypoint_follower'),
             name='waypoint_follower', output='screen', parameters=common, additional_env=nav2_env),
        Node(package='nav2_lifecycle_manager', executable=nav2_executable('nav2_lifecycle_manager', 'lifecycle_manager'),
             name='lifecycle_manager_navigation', output='screen', additional_env=nav2_env,
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'autostart': autostart,
                 # VMware guests can be descheduled for several seconds while
                 # Gazebo loads a large canonical world and controllers start.
                 # Keep Nav2's startup bond from aborting during that transient;
                 # this changes no planning/control behavior.
                 'bond_timeout': bond_timeout,
                 'node_names': ([
                     'map_server', *(['amcl'] if localization_backend == 'AMCL' else []),
                     'controller_server', 'planner_server', 'behavior_server',
                     'bt_navigator', 'waypoint_follower',
                 ] if map_source == 'STATIC_MAP' else [
                     *(['canonical_map_server'] if map_source == 'REGISTERED_CANONICAL' else []),
                     'controller_server', 'planner_server', 'behavior_server',
                     'bt_navigator', 'waypoint_follower',
                 ]),
             }]),
    ])
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_bringup')
    params = os.path.join(pkg_share, 'swerve_navigation', 'config', 'nav2_params.yaml')
    default_map = Path(pkg_share) / 'swerve_navigation' / 'maps' / 'warehouse.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('bond_timeout', default_value='30.0',
                              description='Lifecycle startup heartbeat grace period in seconds.'),
        DeclareLaunchArgument('map_file', default_value=str(default_map), description='Static Nav2 map YAML'),
        DeclareLaunchArgument('map_source', default_value='STATIC_MAP',
                              description='REGISTERED_CANONICAL publishes the selected map on /canonical_map for registration into /navigation_map; STATIC_MAP publishes /navigation_map; LIVE_SLAM consumes /map.'),
        DeclareLaunchArgument('localization_backend', default_value='AMCL',
                              description='AMCL uses the saved map and /scan; SET_POSE is an explicit legacy robot_localization backend.'),
        DeclareLaunchArgument('allow_dev_map', default_value='false',
                              description='Explicitly permit the package development Nav2 map.'),
        DeclareLaunchArgument('params_file', default_value=params, description='Nav2 parameter file'),
        OpaqueFunction(function=lambda context: _nav_nodes(context, params_default=params, default_map=default_map)),
    ])
