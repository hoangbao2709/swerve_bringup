from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('swerve_bridge'), 'config', 'bridge.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_id = LaunchConfiguration('robot_id')
    namespace = LaunchConfiguration('namespace')
    artifact_root = LaunchConfiguration('artifact_root')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file')
    default_artifact_root = os.environ.get('WARETWIN_ARTIFACT_ROOT') or os.path.abspath(
        os.path.join(os.getcwd(), 'generated', 'maps'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_id', default_value='R01'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'runtime_state',
            default_value=EnvironmentVariable('WARETWIN_RUNTIME_STATE', default_value='MAPPING'),
        ),
        DeclareLaunchArgument(
            'django_token',
            default_value=EnvironmentVariable('WARETWIN_ROS_BRIDGE_TOKEN', default_value='')),
        DeclareLaunchArgument(
            'django_ws_url',
            default_value=EnvironmentVariable('ROS_WS_URL', default_value='ws://127.0.0.1:8000/ws/ros'),
        ),
        DeclareLaunchArgument('artifact_root', default_value=default_artifact_root),
        DeclareLaunchArgument('gazebo_world_file', default_value=''),
        Node(package='swerve_bridge', executable='swerve_bridge_node',
             name='swerve_bridge', output='screen',
             parameters=[config, {
                 'use_sim_time': use_sim_time,
                 'robot_id': robot_id,
                 'namespace': namespace,
                 'django_token': LaunchConfiguration('django_token'),
                 'django_ws_url': LaunchConfiguration('django_ws_url'),
                 'runtime_state': LaunchConfiguration('runtime_state'),
                 'artifact_root': artifact_root,
                 'gazebo_world_file': gazebo_world_file,
             }]),
    ])
