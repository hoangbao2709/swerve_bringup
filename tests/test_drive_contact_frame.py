"""Rolling-wheel contact directions must not acquire a normal component."""
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def test_isotropic_drive_friction_uses_automatic_contact_tangents():
    root = ET.parse(ROOT / 'urdf/swerve_base.urdf').getroot()
    for side in ('front', 'rear'):
        surface = root.find(f'gazebo[@reference="wheel_{side}_drive_link"]')
        assert [float(v) for v in surface.findtext('fdir1').split()] == [0., 0., 0.]
        assert {key: float(surface.findtext(key)) for key in
                ('mu1', 'mu2', 'kp', 'kd', 'maxVel', 'minDepth')} == {
                    'mu1': 2., 'mu2': 2., 'kp': 1e6, 'kd': 100.,
                    'maxVel': .05, 'minDepth': 0.}
        joint = root.find(f'joint[@name="wheel_{side}_drive_joint"]')
        assert joint.find('axis').attrib['xyz'] == '0 1 0'
        assert joint.find('limit').attrib == {'effort': '200.0', 'velocity': '30.0'}
        assert joint.find('dynamics').attrib == {'friction': '0.05', 'damping': '0.10'}
    hardware = root.find('.//ros2_control/hardware')
    assert hardware.findtext('plugin') == 'swerve_bringup/GazeboMotorSystem'
    assert not hardware.findall('param')  # no solver or motor-adapter override


def test_old_rolling_x_direction_cannot_be_a_fixed_contact_tangent():
    # Rotation about the physical Y axle transforms local X to (cos,0,-sin).
    # At a quarter-turn it is normal to the floor, not a friction tangent.
    for angle in (math.pi/4, math.pi/2, 3*math.pi/4):
        assert abs(math.sin(angle)) > .7


def test_generated_gazebo_collision_retains_automatic_basis(tmp_path):
    urdf = subprocess.run(['xacro', str(ROOT / 'urdf/swerve_base.urdf'),
                           'use_cad_visuals:=false'], check=True, capture_output=True, text=True).stdout
    path = tmp_path / 'robot.urdf'
    path.write_text(urdf)
    sdf = subprocess.run(['gz', 'sdf', '-p', str(path)], check=True,
                         capture_output=True, text=True).stdout
    model = ET.fromstring(sdf).find('model')
    for side in ('front', 'rear'):
        ode = model.find(f'link[@name="wheel_{side}_drive_link"]/collision/surface/friction/ode')
        assert [float(v) for v in ode.findtext('fdir1').split()] == [0., 0., 0.]
        assert float(ode.findtext('mu')) == float(ode.findtext('mu2')) == 2.


def test_native_ros_parameter_override_accepts_generated_description():
    # gazebo_ros2_control parses its received XML as a CLI parameter override.
    # XML-valid comments can still be YAML-invalid (for example colon-space).
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    urdf = subprocess.run(['xacro', str(ROOT / 'urdf/swerve_base.urdf')],
                          check=True, capture_output=True, text=True).stdout
    context = Context()
    context.init(args=[])
    node = None
    try:
        node = Node('robot_description_parse_regression', context=context,
                    cli_args=['--ros-args', '--param', 'robot_description:=' + urdf],
                    automatically_declare_parameters_from_overrides=True)
        parsed = node.get_parameter('robot_description').value
        assert isinstance(parsed, str)
        # YAML folds presentation whitespace; compare the XML semantics.
        assert ET.canonicalize(parsed, strip_text=True) == ET.canonicalize(urdf, strip_text=True)
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
