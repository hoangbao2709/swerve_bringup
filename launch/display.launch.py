#!/usr/bin/env python3
"""
Launch: hien thi swerve AGV trong RViz.
- robot_state_publisher: publish TF tu URDF
- joint_state_publisher_gui: cho phep keo thanh truot de test tung khop
    (steer_front_joint, steer_rear_joint, wheel_*_drive_joint, wheel_*_swivel_joint, wheel_*_roll_joint)
- rviz2: hien thi mesh + TF

Chay:
    ros2 launch swerve_bringup display.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_bringup')
    default_urdf = os.path.join(pkg_share, 'urdf', 'swerve_base.urdf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'swerve.rviz')

    urdf_arg = DeclareLaunchArgument(
        'urdf_path', default_value=default_urdf,
        description='Duong dan toi file URDF'
    )
    gui_arg = DeclareLaunchArgument(
        'use_joint_state_gui', default_value='true',
        description='Bat/tat joint_state_publisher_gui (thanh truot test khop)'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use a simulation clock when displaying a live Gazebo robot',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        urdf_arg,
        gui_arg,
        sim_time_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
