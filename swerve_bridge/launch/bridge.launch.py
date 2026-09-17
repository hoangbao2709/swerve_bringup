from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('swerve_bridge'), 'config', 'bridge.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'django_token',
            default_value=EnvironmentVariable('WARETWIN_ROS_BRIDGE_TOKEN', default_value='')),
        DeclareLaunchArgument('django_ws_url', default_value='ws://127.0.0.1:8000/ws/ros'),
        Node(package='swerve_bridge', executable='swerve_bridge_node',
             name='swerve_bridge', output='screen',
             parameters=[config, {
                 'use_sim_time': use_sim_time,
                 'django_token': LaunchConfiguration('django_token'),
                 'django_ws_url': LaunchConfiguration('django_ws_url'),
             }]),
    ])
