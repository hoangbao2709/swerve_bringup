from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def test_adapter_preserves_native_steering_read_and_original_resource_storage():
    source = (ROOT / 'ros_compat/gazebo_motor_system.cpp').read_text()
    assert 'return upstream_->read(time, period);' in source
    assert 'return upstream_->export_state_interfaces();' in source
    assert 'auto interfaces = upstream_->export_command_interfaces();' in source
    assert 'upstream_->write(time, period)' in source
    assert 'createSharedInstance("gazebo_ros2_control/GazeboSystem")' in source
    assert 'if (interface.name != "velocity") {continue;}' in source
    assert 'position_motor_velocity' not in source
    assert 'Physics()->SetParam' not in source  # no solver-profile mutation
    root = ET.parse(ROOT / 'urdf/swerve_base.urdf').getroot()
    control = root.find('.//ros2_control')
    assert {j.attrib['name']: j.find('command_interface').attrib['name']
            for j in control.findall('joint')} == {
                'steer_front_joint': 'position', 'steer_rear_joint': 'position',
                'wheel_front_drive_joint': 'velocity', 'wheel_rear_drive_joint': 'velocity'}
    assert all(not j.findall('param') for j in control.findall('joint'))


def test_adapter_export_uses_existing_hardware_plugin_loader():
    plugin = ET.parse(ROOT / 'config/gazebo_motor_plugin.xml').getroot()
    assert plugin.attrib['path'] == 'swerve_gazebo_motor_system'
    assert plugin.find('class').attrib == {
        'name': 'swerve_bringup/GazeboMotorSystem', 'type': 'swerve_bringup::GazeboMotorSystem',
        'base_class_type': 'gazebo_ros2_control::GazeboSystemInterface'}
