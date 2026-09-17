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
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
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
    lidar_topic = LaunchConfiguration('real_lidar_topic')
    imu_topic = LaunchConfiguration('real_imu_topic')
    odom_topic = LaunchConfiguration('real_odom_topic')
    urdf = os.path.join(pkg, 'urdf', 'swerve_base.urdf')
    interface_cfg = os.path.join(pkg, 'config', 'sim_real_interface.yaml')

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
    robot_description = ParameterValue(Command(['xacro ', urdf] + args), value_type=str)

    # gazebo.launch.py owns Gazebo, spawn, ros2_control and simulated sensors.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(use_sim),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
                          'use_sim_time': use_sim_time,
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
                                   launch_arguments={'use_sim_time': use_sim_time, 'input_topic': lidar_topic}.items())
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
                                  launch_arguments={'use_sim_time': use_sim_time}.items())
    bridge = Node(package='swerve_bridge', executable='swerve_bridge_node', name='swerve_bridge', output='screen',
                  parameters=[os.path.join(get_package_share_directory('swerve_bridge'), 'config', 'bridge.yaml'),
                              {'use_sim_time': use_sim_time,
                               'django_token': LaunchConfiguration('bridge_token'),
                               'django_ws_url': LaunchConfiguration('bridge_ws_url')}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='true=Gazebo, false=physical robot drivers'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo clock; set false for real robot'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'warehouse.world')),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo client window'),
        DeclareLaunchArgument('real_sensor_launch', default_value='', description='Vendor sensor launch file for use_sim=false'),
        DeclareLaunchArgument('real_lidar_topic', default_value='/lidar/points'),
        DeclareLaunchArgument('real_imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('real_odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'bridge_token',
            default_value=EnvironmentVariable('WARETWIN_ROS_BRIDGE_TOKEN', default_value=''),
            description='Token for the Django ROS bridge (defaults to WARETWIN_ROS_BRIDGE_TOKEN)'),
        DeclareLaunchArgument('bridge_ws_url', default_value='ws://127.0.0.1:8000/ws/ros'),
        sim, real_driver, real_state_publisher, ekf, slam, nav, bridge,
    ])
