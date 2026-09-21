#!/usr/bin/env python3
"""Unified simulation/real-robot bringup.

Both modes expose the same downstream topics and frames:
  /lidar/points -> /lidar/points_filtered -> /scan
  /imu/data + /odom -> robot_localization -> /odometry/filtered

For a real robot, pass a driver launch file with ``real_sensor_launch``. That
launch is responsible only for starting the vendor drivers; its topics are
remapped to the standard interface below.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
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
    caster_axle_offset_x_m = LaunchConfiguration('caster_axle_offset_x_m')
    caster_axle_offset_y_m = LaunchConfiguration('caster_axle_offset_y_m')
    proper_caster_dynamics = {name: LaunchConfiguration(name) for name in (
        'proper_caster_mu1', 'proper_caster_mu2', 'caster_swivel_friction',
        'caster_swivel_damping', 'caster_roll_friction', 'caster_roll_damping')}
    mode = LaunchConfiguration('mode')
    mapping_mode = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    navigation_mode = IfCondition(PythonExpression(["'", mode, "' == 'navigation'"]))
    lidar_topic = LaunchConfiguration('real_lidar_topic')
    imu_topic = LaunchConfiguration('real_imu_topic')
    odom_topic = LaunchConfiguration('real_odom_topic')
    artifact_root = LaunchConfiguration('artifact_root')
    robot_id = LaunchConfiguration('robot_id')
    namespace = LaunchConfiguration('namespace')
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

    mode_guard = OpaqueFunction(function=validate_mode)

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
    for name, value in proper_caster_dynamics.items(): args += [f' {name}:=', value]
    robot_description = ParameterValue(Command(['xacro ', urdf] + args), value_type=str)

    # gazebo.launch.py owns Gazebo, spawn, ros2_control and simulated sensors.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(use_sim),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
                          'robot_id': robot_id,
                          'use_sim_time': use_sim_time,
                          'contact_diagnostics': contact_diagnostics,
                          'caster_frictionless': caster_frictionless,
                          'proper_caster_test': proper_caster_test,
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
                                   launch_arguments={'use_sim_time': use_sim_time, 'input_topic': lidar_topic,
                                                     'start_slam': 'true'}.items(),
                                   condition=mapping_mode)
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
                                  launch_arguments={'use_sim_time': use_sim_time,
                                                    'map_file': map_file}.items(),
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
                               'gazebo_world_file': LaunchConfiguration('world')}])
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
                          'tag_graph_file': tag_graph_file}.items(),
        condition=navigation_mode)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='true=Gazebo, false=physical robot drivers'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo clock; set false for real robot'),
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping=SLAM owns map->odom; navigation=static map + V30E owns map->odom'),
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
        mode_guard, sim, real_driver, real_state_publisher, ekf, v30e, slam, nav, bridge, rviz,
    ])
