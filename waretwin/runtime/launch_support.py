"""Shared launch argument and process contract for web and full-stack modes."""
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def description(full_stack=False):
    defaults = dict(backend_host='', backend_port='', frontend_host='', frontend_port='',
                    ros_domain_id='', config_file='', runtime_dir='', backend_python='',
                    startup_timeout='120')
    if full_stack:
        defaults.update(mode='unified', use_sim='true', allow_dev_world='false', robot_id='R01',
                        namespace='', gui='false', start_rviz='false', real_sensor_launch='', ros_timeout='600')

    def start(context):
        share = Path(get_package_share_directory('waretwin_web'))
        command = [sys.executable, str(share / 'runtime/service.py'), '--share', str(share)]
        if full_stack:
            command.append('--full-stack')
        for key in defaults:
            command.extend(('--' + key.replace('_', '-'), LaunchConfiguration(key).perform(context)))
        process = ExecuteProcess(cmd=command, output='screen', sigterm_timeout='60', sigkill_timeout='10')

        def exited(event, _context):
            if event.returncode != 0 and not _context.is_shutdown:
                raise RuntimeError(f'WareTwin essential runtime exited with status {event.returncode}')
            return [EmitEvent(event=Shutdown(reason='WareTwin runtime stopped'))]

        return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=exited)), process]

    return LaunchDescription([*(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
                              OpaqueFunction(function=start)])
