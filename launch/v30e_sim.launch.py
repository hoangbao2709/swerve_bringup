#!/usr/bin/env python3
"""V30E simulation contract and map->odom correction filter."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    def ekf_from_spawn(context):
        x = float(LaunchConfiguration('initial_x').perform(context))
        y = float(LaunchConfiguration('initial_y').perform(context))
        yaw = float(LaunchConfiguration('initial_yaw').perform(context))
        # robot_localization state ordering is x,y,z,roll,pitch,yaw,vx,vy,vz,
        # vroll,vpitch,vyaw,ax,ay,az. Only the map-frame spawn prior is set;
        # encoder odometry remains local and starts at zero in `odom`.
        initial_state = [x, y, 0.0, 0.0, 0.0, yaw] + [0.0] * 9
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        return [Node(
            package='robot_localization', executable='ekf_node', name='ekf_v30e',
            output='screen', parameters=[ekf_cfg, {
                'use_sim_time': use_sim_time,
                'initial_state': initial_state,
            }], remappings=[
                ('odometry/filtered', '/odometry/v30e'),
                # The common odom-frame EKF also advertises robot_localization's
                # default /set_pose service. Give this map-frame filter an
                # unambiguous endpoint so Initial Pose cannot reset the wrong
                # filter after a supervised Navigation transition.
                ('set_pose', '/ekf_v30e/set_pose'),
            ],
        )]

    ekf = OpaqueFunction(function=ekf_from_spawn, condition=IfCondition(enabled))
    tag_navigation = Node(package='swerve_bringup', executable='tag_route_planner',
                          name='tag_route_planner', output='screen', condition=IfCondition(enabled),
                          parameters=[tag_nav_cfg, {'tag_graph_file': tag_graph,
                                                    'use_sim_time': LaunchConfiguration('use_sim_time')}])
    return LaunchDescription([
        DeclareLaunchArgument('enable_v30e_sim', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('datamatrix_map_file', default_value=map_file),
        DeclareLaunchArgument('tag_graph_file', default_value=os.path.join(pkg, 'config', 'tag_graph.yaml')),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        reader, tag_navigation, ekf])
