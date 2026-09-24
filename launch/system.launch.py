#!/usr/bin/env python3
"""Unified simulation/real-robot bringup.

Both modes expose the same downstream topics and frames:
  /lidar/points -> /lidar/points_filtered -> /scan
  /imu/data + /odom -> robot_localization -> /odometry/filtered

For a real robot, pass a driver launch file with ``real_sensor_launch``. That
launch is responsible only for starting the vendor drivers; its topics are
remapped to the standard interface below.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _real_driver(context, package_share):
    path = LaunchConfiguration('real_sensor_launch').perform(context).strip()
    if not path:
        return [LogInfo(msg='use_sim=false but real_sensor_launch is empty; start/remap the physical sensor drivers separately.')]
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]


def generate_launch_description():
    pkg = get_package_share_directory('swerve_bringup')
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    contact_diagnostics = LaunchConfiguration('contact_diagnostics')
    caster_frictionless = LaunchConfiguration('caster_frictionless')
    proper_caster_test = LaunchConfiguration('proper_caster_test')
    use_cad_visuals = LaunchConfiguration('use_cad_visuals')
    caster_axle_offset_x_m = LaunchConfiguration('caster_axle_offset_x_m')
    caster_axle_offset_y_m = LaunchConfiguration('caster_axle_offset_y_m')
    proper_caster_dynamics = {name: LaunchConfiguration(name) for name in (
        'proper_caster_mu1', 'proper_caster_mu2', 'caster_swivel_friction',
        'caster_swivel_damping', 'caster_roll_friction', 'caster_roll_damping')}
    mode = LaunchConfiguration('mode')
    mapping_mode = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    navigation_mode = IfCondition(PythonExpression(["'", mode, "' == 'navigation'"]))
    simulated_mapping_mode = IfCondition(PythonExpression([
        "'", use_sim, "' == 'true' and '", mode, "' == 'mapping'"]))
    require_canonical_map = ParameterValue(PythonExpression([
        "'", use_sim, "' == 'true' or '", mode, "' == 'navigation'"]), value_type=bool)
    require_tag_map = ParameterValue(PythonExpression([
        "'", use_sim, "' == 'true' or '", mode, "' == 'navigation'"]), value_type=bool)
    lidar_topic = LaunchConfiguration('real_lidar_topic')
    imu_topic = LaunchConfiguration('real_imu_topic')
    odom_topic = LaunchConfiguration('real_odom_topic')
    artifact_root = LaunchConfiguration('artifact_root')
    robot_id = LaunchConfiguration('robot_id')
    namespace = LaunchConfiguration('namespace')
    allow_dev_world = LaunchConfiguration('allow_dev_world')
    map_sync_request_file = LaunchConfiguration('map_sync_request_file')
    datamatrix_map_file = LaunchConfiguration('datamatrix_map_file')
    tag_graph_file = LaunchConfiguration('tag_graph_file')
    map_file = LaunchConfiguration('map_file')
    urdf = os.path.join(pkg, 'urdf', 'swerve_base.urdf')
    interface_cfg = os.path.join(pkg, 'config', 'sim_real_interface.yaml')
    default_artifact_root = os.environ.get('WARETWIN_ARTIFACT_ROOT') or os.path.abspath(
        os.path.join(os.getcwd(), 'generated', 'maps'))
    default_rviz_config = os.path.join(pkg, 'rviz', 'swerve.rviz')

    def validate_mode(context):
        selected = LaunchConfiguration('mode').perform(context).strip().lower()
        if selected not in ('mapping', 'navigation'):
            raise RuntimeError(
                f'Unsupported mode={selected!r}; choose exactly mapping or navigation '
                '(SLAM and Nav2 are mutually exclusive)')
        return [LogInfo(msg=f'WareTwin runtime mode: {selected.upper()}')]

    def validate_map_bundle(context):
        if LaunchConfiguration('use_sim').perform(context).lower() != 'true':
            return []
        if LaunchConfiguration('allow_dev_world').perform(context).lower() == 'true':
            return [LogInfo(msg='Explicit development-world fallback is enabled')]
        world = Path(LaunchConfiguration('world').perform(context)).expanduser().resolve()
        artifact_root = world.parent.parent
        manifest_path = artifact_root / 'manifest.json'
        try:
            manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
            revision = int(manifest['revision'])
            if manifest.get('frame_id') != 'map' or manifest.get('units') != 'm':
                raise ValueError('manifest must declare frame_id=map and units=m')
            if world != (artifact_root / manifest['artifacts']['gazebo_world']).resolve():
                raise ValueError('selected Gazebo world is not the manifest world artifact')
            world_xml = ET.parse(world).getroot()
            if world_xml.find(".//plugin[@name='gazebo_ros_state'][@filename='libgazebo_ros_state.so']") is None:
                raise ValueError('Gazebo world lacks gazebo_ros_state plugin required for V30E simulation')
            wanted = str(LaunchConfiguration('robot_id').perform(context))
            robot = next((row for row in manifest.get('robots', []) if str(row.get('id')) == wanted), None)
            if robot is None:
                raise ValueError(f'robot {wanted} has no canonical spawn pose')
            spawn = robot.get('pose')
            if not isinstance(spawn, list) or len(spawn) != 4:
                raise ValueError(f'robot {wanted} spawn pose must be [x, y, z, yaw]')
            spawn = [float(value) for value in spawn]
            if not all(math.isfinite(value) for value in spawn):
                raise ValueError(f'robot {wanted} spawn pose contains a non-finite value')
            nav2_rel = (manifest.get('nav2_maps') or {}).get(str(robot.get('floor_id')))
            if not nav2_rel:
                nav2_rel = manifest['artifacts']['nav2_map']
            expected = {
                'datamatrix_map_file': (artifact_root / manifest['artifacts']['datamatrix_map']).resolve(),
                'tag_graph_file': (artifact_root / manifest['artifacts']['tag_graph']).resolve(),
            }
            expected['map_file'] = (artifact_root / nav2_rel).resolve()
            for arg, path in expected.items():
                actual = Path(LaunchConfiguration(arg).perform(context)).expanduser().resolve()
                if actual != path:
                    raise ValueError(f'{arg} is not from published revision {revision}: {actual}')
            for relative, expected_hash in (manifest.get('sha256') or {}).items():
                path = (artifact_root / relative).resolve(strict=True)
                if not path.is_relative_to(artifact_root):
                    raise ValueError(f'artifact path escapes revision bundle: {relative}')
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                if digest != expected_hash:
                    raise ValueError(f'artifact hash mismatch: {relative}')
            canonical = json.loads((artifact_root / manifest['artifacts']['canonical_map']).read_text(encoding='utf-8'))
            if canonical.get('frame_id') != 'map' or int(canonical.get('revision', -1)) != revision:
                raise ValueError('canonical map frame/revision does not match manifest')
            origin = canonical.get('origin') or {}
            expected_bounds = {
                'min_x': float(origin['x']), 'min_y': float(origin['y']),
                'max_x': float(origin['x']) + float(canonical['width']),
                'max_y': float(origin['y']) + float(canonical['height']),
            }
            actual_bounds = manifest.get('gazebo_bounds') or {}
            if any(abs(float(actual_bounds[key]) - value) > 1e-6 for key, value in expected_bounds.items()):
                raise ValueError('Gazebo world bounds do not match canonical map bounds')
            gazebo_manifest = json.loads(
                (artifact_root / manifest['artifacts']['gazebo_manifest']).read_text(encoding='utf-8'))
            canonical_floors = {str(item.get('id')): item for item in canonical.get('floors', [])}
            gazebo_floors = {str(item.get('id')): item for item in gazebo_manifest.get('floors', [])}
            if set(canonical_floors) != set(gazebo_floors):
                raise ValueError('Gazebo floor set does not match canonical map')
            for floor_id, floor in canonical_floors.items():
                if (floor.get('boundary') != gazebo_floors[floor_id].get('boundary')
                        or floor.get('holes', []) != gazebo_floors[floor_id].get('holes', [])):
                    raise ValueError(f'Gazebo floor {floor_id} geometry does not match canonical map')
            floor = canonical_floors.get(str(robot.get('floor_id')))
            nav_bounds = (manifest.get('nav2_bounds') or {}).get(str(robot.get('floor_id')))
            if floor is None or nav_bounds is None:
                raise ValueError('Nav2 bounds are missing for selected robot floor')
            ring = floor.get('boundary') or []
            xs = [float(point['x'] if isinstance(point, dict) else point[0]) for point in ring]
            ys = [float(point['y'] if isinstance(point, dict) else point[1]) for point in ring]
            resolution = float(nav_bounds['resolution'])
            nav_origin = nav_bounds['origin']
            nav_size = (float(nav_bounds['width']) * resolution,
                        float(nav_bounds['height']) * resolution)
            if (abs(float(nav_origin[0]) - min(xs)) > 1e-6
                    or abs(float(nav_origin[1]) - min(ys)) > 1e-6
                    or resolution <= 0
                    or nav_size[0] + 1e-6 < max(xs) - min(xs)
                    or nav_size[1] + 1e-6 < max(ys) - min(ys)
                    or nav_size[0] - (max(xs) - min(xs)) >= resolution + 1e-6
                    or nav_size[1] - (max(ys) - min(ys)) >= resolution + 1e-6):
                raise ValueError('Nav2 bounds/origin do not match canonical floor')
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f'Published map bundle validation failed before simulation startup: {exc}. '
                'Publish a valid map or explicitly set allow_dev_world:=true.') from exc
        return [
            SetLaunchConfiguration('v30e_initial_x', str(spawn[0])),
            SetLaunchConfiguration('v30e_initial_y', str(spawn[1])),
            SetLaunchConfiguration('v30e_initial_yaw', str(spawn[3])),
            LogInfo(msg=f'Published canonical map bundle verified: revision={revision}, frame=map'),
            LogInfo(msg=f'Global localization prior loaded from robot spawn: x={spawn[0]}, y={spawn[1]}, yaw={spawn[3]}'),
        ]

    mode_guard = OpaqueFunction(function=validate_mode)
    map_guard = OpaqueFunction(function=validate_map_bundle)

    # Keep URDF calibration in one place. Gazebo consumes the same values for
    # its sensor plugins; on a real robot the fixed TF is published here.
    import yaml
    with open(interface_cfg, encoding='utf-8') as stream:
        interface = yaml.safe_load(stream)
        lidar = interface['lidar']
        lidar_extrinsics = interface['lidar_extrinsics']
        imu_extrinsics = interface['imu_extrinsics']
    with open(interface_cfg, encoding='utf-8') as stream:
        imu = yaml.safe_load(stream)['imu']

    def xacro_arg(name, value):
        return [f' {name}:={value}']

    args = []
    for name, value in zip(('lidar_x', 'lidar_y', 'lidar_z'), lidar_extrinsics['xyz']): args += xacro_arg(name, value)
    for name, value in zip(('lidar_roll', 'lidar_pitch', 'lidar_yaw'), lidar_extrinsics['rpy']): args += xacro_arg(name, value)
    for name, value in zip(('imu_x', 'imu_y', 'imu_z'), imu_extrinsics['xyz']): args += xacro_arg(name, value)
    for name, value in zip(('imu_roll', 'imu_pitch', 'imu_yaw'), imu_extrinsics['rpy']): args += xacro_arg(name, value)
    args += [' caster_axle_offset_x_m:=', caster_axle_offset_x_m]
    args += [' caster_axle_offset_y_m:=', caster_axle_offset_y_m]
    args += [' use_cad_visuals:=', use_cad_visuals]
    for name, value in proper_caster_dynamics.items(): args += [f' {name}:=', value]
    robot_description = ParameterValue(Command(['xacro ', urdf] + args), value_type=str)

    # gazebo.launch.py owns Gazebo, spawn, ros2_control and simulated sensors.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(use_sim),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
                          'allow_dev_world': allow_dev_world,
                          'robot_id': robot_id,
                          'use_sim_time': use_sim_time,
                          'contact_diagnostics': contact_diagnostics,
                          'caster_frictionless': caster_frictionless,
                          'proper_caster_test': proper_caster_test,
                          'use_cad_visuals': use_cad_visuals,
                          'caster_axle_offset_x_m': caster_axle_offset_x_m,
                          'caster_axle_offset_y_m': caster_axle_offset_y_m,
                          **proper_caster_dynamics,
                          'start_ekf': 'false',
                          'start_state_publisher': 'true'}.items())
    real_driver = OpaqueFunction(function=lambda context: _real_driver(context, pkg), condition=UnlessCondition(use_sim))
    real_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                                name='robot_state_publisher', output='screen',
                                condition=UnlessCondition(use_sim),
                                parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}])

    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
               parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
               remappings=[('/odom', odom_topic), ('/imu/data', imu_topic)])
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
                                   launch_arguments={
                                       'use_sim_time': use_sim_time, 'input_topic': lidar_topic,
                                       'start_slam': 'true',
                                       'map_topic': PythonExpression([
                                           "'/slam/map' if '", use_sim, "' == 'true' else '/map'"]),
                                       'transform_publish_period': PythonExpression([
                                           "'0.0' if '", use_sim, "' == 'true' else '0.02'"]),
                                   }.items(),
                                   condition=mapping_mode)
    # The point-cloud preprocessor and 2D projection are needed by Nav2 too.
    # Only SLAM itself is mapping-only; navigation gets the same /scan
    # contract while V30E owns map -> odom.
    navigation_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'input_topic': lidar_topic,
                          'start_slam': 'false'}.items(),
        condition=navigation_mode)
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
                                  launch_arguments={'use_sim_time': use_sim_time,
                                                    'map_file': map_file,
                                                    'allow_dev_map': allow_dev_world}.items(),
                                  condition=navigation_mode)
    bridge = Node(package='swerve_bridge', executable='swerve_bridge_node', name='swerve_bridge', output='screen',
                  parameters=[os.path.join(get_package_share_directory('swerve_bridge'), 'config', 'bridge.yaml'),
                              {'use_sim_time': use_sim_time,
                               'robot_id': robot_id,
                               'namespace': namespace,
                               'runtime_state': mode,
                               'django_token': LaunchConfiguration('bridge_token'),
                               'django_ws_url': LaunchConfiguration('bridge_ws_url'),
                               'artifact_root': artifact_root,
                               'gazebo_world_file': LaunchConfiguration('world'),
                               'nav2_map_file': map_file,
                               'datamatrix_map_file': datamatrix_map_file,
                               'tag_graph_file': tag_graph_file,
                               'map_sync_request_file': map_sync_request_file,
                               'require_nav2_map': require_canonical_map,
                               'require_tag_map': require_tag_map}])
    mapping_map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_file}],
        condition=simulated_mapping_mode,
    )
    mapping_map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_mapping_map', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server']}],
        condition=simulated_mapping_mode,
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )
    v30e = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'v30e_sim.launch.py')),
        launch_arguments={'enable_v30e_sim': 'true', 'use_sim_time': use_sim_time,
                          'datamatrix_map_file': datamatrix_map_file,
                          'tag_graph_file': tag_graph_file,
                          'initial_x': LaunchConfiguration('v30e_initial_x'),
                          'initial_y': LaunchConfiguration('v30e_initial_y'),
                          'initial_yaw': LaunchConfiguration('v30e_initial_yaw')}.items(),
        condition=IfCondition(use_sim))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='true=Gazebo, false=physical robot drivers'),
        DeclareLaunchArgument('allow_dev_world', default_value='false',
                              description='Explicitly permit development world/map fallback.'),
        DeclareLaunchArgument('map_sync_request_file', default_value='.runtime/map-sync-request.json'),
        DeclareLaunchArgument('v30e_initial_x', default_value='0.0'),
        DeclareLaunchArgument('v30e_initial_y', default_value='0.0'),
        DeclareLaunchArgument('v30e_initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo clock; set false for real robot'),
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='Sim mapping uses canonical map + tag localization; real mapping uses SLAM; navigation uses canonical map + tag localization.'),
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(pkg, 'swerve_navigation', 'maps', 'warehouse.yaml'),
            description='Static Nav2 map YAML; used only when mode=navigation',
        ),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'warehouse.world')),
        DeclareLaunchArgument('robot_id', default_value='R01', description='Robot ID selected from the published Gazebo manifest'),
        DeclareLaunchArgument('namespace', default_value='', description='Optional ROS namespace for the bridge/topic contract'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo client window'),
        DeclareLaunchArgument('start_rviz', default_value='true', description='Start RViz with the project display configuration'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config, description='RViz display configuration'),
        DeclareLaunchArgument('contact_diagnostics', default_value='false', description='Enable temporary Gazebo contact sensors'),
        DeclareLaunchArgument('caster_frictionless', default_value='false', description='Test-only caster friction A/B variant'),
        DeclareLaunchArgument('proper_caster_test', default_value='false', description='Test-only four-caster swivel+roll model'),
        DeclareLaunchArgument('use_cad_visuals', default_value='true',
                              description='Use optimized CAD-derived visual meshes; false selects visual-only primitives'),
        DeclareLaunchArgument('caster_axle_offset_x_m', default_value='0.0', description='Test-only caster axle offset in local +X (m)'),
        DeclareLaunchArgument('caster_axle_offset_y_m', default_value='0.0', description='Test-only caster axle offset in local +Y (m)'),
        DeclareLaunchArgument('proper_caster_mu1', default_value='0.01', description='TEST-ONLY proper caster mu1'),
        DeclareLaunchArgument('proper_caster_mu2', default_value='0.01', description='TEST-ONLY proper caster mu2'),
        DeclareLaunchArgument('caster_swivel_friction', default_value='0.02', description='TEST-ONLY caster swivel friction'),
        DeclareLaunchArgument('caster_swivel_damping', default_value='0.02', description='TEST-ONLY caster swivel damping'),
        DeclareLaunchArgument('caster_roll_friction', default_value='0.02', description='TEST-ONLY caster roll friction'),
        DeclareLaunchArgument('caster_roll_damping', default_value='0.02', description='TEST-ONLY caster roll damping'),
        DeclareLaunchArgument('real_sensor_launch', default_value='', description='Vendor sensor launch file for use_sim=false'),
        DeclareLaunchArgument('real_lidar_topic', default_value='/lidar/points'),
        DeclareLaunchArgument('real_imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('real_odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'bridge_token',
            default_value=EnvironmentVariable('WARETWIN_ROS_BRIDGE_TOKEN', default_value=''),
            description='Token for the Django ROS bridge (defaults to WARETWIN_ROS_BRIDGE_TOKEN)'),
        DeclareLaunchArgument(
            'bridge_ws_url',
            default_value=EnvironmentVariable('ROS_WS_URL', default_value='ws://127.0.0.1:8000/ws/ros'),
            description='Django ROS bridge WebSocket URL.',
        ),
        DeclareLaunchArgument('artifact_root', default_value=default_artifact_root,
                              description='Published map artifact root used by the ROS bridge'),
        DeclareLaunchArgument('datamatrix_map_file', default_value=os.path.join(pkg, 'config', 'datamatrix_map.yaml'),
                              description='Published DataMatrix YAML; package config is the development fallback'),
        DeclareLaunchArgument('tag_graph_file', default_value=os.path.join(pkg, 'config', 'tag_graph.yaml'),
                              description='Published tag graph YAML; package config is the development fallback'),
        mode_guard, map_guard, sim, real_driver, real_state_publisher, ekf,
        v30e, slam, navigation_lidar, nav, mapping_map_server, mapping_map_lifecycle,
        bridge, rviz,
    ])
