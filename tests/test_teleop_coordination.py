import math
import sys
import queue
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'swerve_controller'))
sys.path.insert(0, str(ROOT / 'swerve_bridge'))
from steering_alignment import alignment_scale
from swerve_bridge.control_mailbox import ControlMailbox


def test_alignment_is_symmetric_continuous_and_holds_unaligned_wheels():
    for angle in (0, .1, .3, .7, math.pi/2, math.pi):
        assert alignment_scale(angle,.1,.7) == pytest.approx(alignment_scale(-angle,.1,.7))
    assert alignment_scale(.1,.1,.7) == 1
    assert alignment_scale(.7,.1,.7) == 0
    assert alignment_scale(.7-1e-5,.1,.7) < 1e-8
    assert 0 < alignment_scale(.4,.1,.7) < 1
    with pytest.raises(ValueError): alignment_scale(0,.7,.1)


@pytest.mark.parametrize('actual,expected_moving', [(0.0, False), (math.pi / 2, True)])
def test_controller_gates_drive_against_final_angle_not_ramped_target(actual, expected_moving):
    from swerve_controller_node import SwerveController
    from rclpy.time import Time
    output = []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=1100000000)),
        emergency_stop=False, last_cmd_time=1.0, command_timeout=0.5,
        last_update_time=Time(nanoseconds=1000000000),
        modules=('front', 'rear'), targets={m: (math.pi / 2, 3.0) for m in ('front', 'rear')},
        steering_state={m: actual for m in ('front', 'rear')},
        steering_command={m: 0.0 for m in ('front', 'rear')},
        wheel_command={m: 0.0 for m in ('front', 'rear')},
        alignment_full_error=0.1, alignment_stop_error=0.7,
        max_steering_rate=2.0, max_wheel_velocity=10.0, max_wheel_acceleration=10.0,
        steer_pub=SimpleNamespace(publish=lambda _: None),
        drive_pub=SimpleNamespace(publish=lambda msg: output.append(list(msg.data))))
    SwerveController.update(node)
    assert all((abs(value) > 0) == expected_moving for value in output[-1])


def test_low_speed_nav_yaw_correction_survives_module_deadband():
    from geometry_msgs.msg import Twist
    from swerve_controller_node import SwerveController

    config = yaml.safe_load((ROOT / 'config/swerve_controller.yaml').read_text())
    params = config['swerve_controller']['ros__parameters']
    wheel_radius = params['wheel_radius']
    module_x = params['modules.front.x']
    nav_yaw_command = 0.0158
    module_linear_speed = abs(nav_yaw_command * module_x)
    assert module_linear_speed > params['speed_deadband']

    node = SimpleNamespace(
        emergency_stop=False,
        last_cmd_time=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_100_000_000)),
        wheel_radius=wheel_radius,
        max_linear_velocity=params['max_linear_velocity'],
        max_angular_velocity=params['max_angular_velocity'],
        max_steering_angle=params['max_steering_angle'],
        speed_deadband=params['speed_deadband'],
        modules=('front', 'rear'),
        module_xy={
            'front': (params['modules.front.x'], params['modules.front.y']),
            'rear': (params['modules.rear.x'], params['modules.rear.y']),
        },
        steering_command={'front': 0.0, 'rear': 0.0},
        steering_state={'front': 0.0, 'rear': 0.0},
    )
    command = Twist()
    command.angular.z = nav_yaw_command
    SwerveController.cmd_vel_callback(node, command)

    for _, wheel_speed in node.targets.values():
        assert wheel_speed > 0.0
        assert wheel_speed * wheel_radius > params['speed_deadband']


@pytest.mark.parametrize('barrier', [
    {'type':'MANUAL_CMD','action':'STOP'}, {'type':'CONTROL_MODE','mode':'AUTONOMOUS'},
    {'type':'EMERGENCY_STOP'}, {'type':'CLEAR_EMERGENCY_STOP'}, {'type':'MANUAL_DISCONNECT'},
])
def test_barriers_invalidate_queued_and_already_taken_motion(barrier):
    mailbox=ControlMailbox()
    mailbox.put({'type':'MANUAL_CMD','action':'FORWARD'})
    old=mailbox.get_nowait()
    for _ in range(1000): mailbox.put({'type':'MANUAL_CMD','action':'LEFT'})
    mailbox.put(barrier)
    assert not mailbox.current(old)
    assert mailbox.manual is None
    assert mailbox.get_nowait()['type']==barrier['type']
    with pytest.raises(queue.Empty): mailbox.get_nowait()


def test_latest_manual_replaces_pending_without_reordering_mode_barrier():
    m=ControlMailbox();m.put({'type':'CONTROL_MODE','mode':'MANUAL'})
    m.put({'type':'MANUAL_CMD','action':'FORWARD'})
    m.put({'type':'MANUAL_CMD','action':'BACKWARD'})
    assert m.get_nowait()['type']=='CONTROL_MODE'
    latest=m.get_nowait();assert latest['action']=='BACKWARD' and m.current(latest)


def test_bridge_does_not_apply_expired_or_invalidated_manual():
    from swerve_bridge.bridge_node import SwerveBridge
    import time
    m=ControlMailbox();m.put({'type':'MANUAL_CMD','action':'FORWARD'});data=m.get_nowait()
    data['_received_monotonic']=time.monotonic()-2
    bridge=SimpleNamespace(incoming=m,get_parameter=lambda _:SimpleNamespace(value=.4),mode_transition_state='APPLIED')
    SwerveBridge._apply_manual_command(bridge,data)
    m.put({'type':'MANUAL_CMD','action':'STOP'})
    data['_received_monotonic']=time.monotonic()
    SwerveBridge._apply_manual_command(bridge,data)


def test_mode_confirmation_requires_matching_arbiter_request():
    from swerve_bridge.bridge_node import SwerveBridge
    import json
    sent=[]
    bridge=SimpleNamespace(command_diagnostics={},applied_mode='MANUAL',requested_mode='AUTONOMOUS',
        mode_request_id='new',mode_transition_state='REQUESTED',robot_id='R01',
        send=lambda _:None,now=lambda:'now',send_control_status=lambda _:sent.append(True))
    SwerveBridge.command_diagnostics_cb(bridge,SimpleNamespace(data=json.dumps({'active_control_mode':'AUTONOMOUS','control_mode_request_id':'old'})))
    assert bridge.mode_transition_state=='REQUESTED' and not sent
    SwerveBridge.command_diagnostics_cb(bridge,SimpleNamespace(data=json.dumps({'active_control_mode':'AUTONOMOUS','control_mode_request_id':'new'})))
    assert bridge.mode_transition_state=='APPLIED' and sent==[True]


def test_generation_invalidation_emits_final_zero_then_releases_idle_source():
    from swerve_bridge.bridge_node import SwerveBridge
    from geometry_msgs.msg import Twist
    from command_ownership import choose_command
    mailbox = ControlMailbox()
    mailbox.put({'type': 'MANUAL_DISCONNECT'})
    old = Twist()
    old.linear.x = .25
    sent = []
    bridge = SimpleNamespace(incoming=mailbox, manual_generation=0,
        mode_transition_state='APPLIED', manual_twist=old, manual_deadline=999.,
        emergency_stop_active=False, control_mode='MANUAL', local_map_load_pending=False,
        trace_control_callback=lambda _: None,
        cmd_pub=SimpleNamespace(publish=sent.append))
    bridge._manual_timer = lambda: SwerveBridge._manual_timer(bridge)
    for _ in range(20):
        SwerveBridge.manual_timer(bridge)
    assert len(sent) == 1
    assert sent[0].linear.x == 0 and bridge.manual_deadline == 0
    assert bridge.manual_generation == mailbox.generation
    owner, values = choose_command(now=1., timeout=.5, mode='MANUAL',
        sources={'WEB_MANUAL': ((0., 0., 0.), 0.)})
    assert owner == 'NONE' and values == (0., 0., 0.)


def test_lease_timer_applies_fresh_pending_ingress_before_old_expiry(monkeypatch):
    from swerve_bridge.bridge_node import SwerveBridge
    from geometry_msgs.msg import Twist
    import time
    mailbox = ControlMailbox()
    mailbox.put({'type': 'MANUAL_CMD', 'action': 'FORWARD'})
    mailbox.manual['_received_monotonic'] = .39
    monkeypatch.setattr(time, 'monotonic', lambda: .41)
    sent = []
    bridge = SimpleNamespace(incoming=mailbox, manual_generation=0,
        mode_transition_state='APPLIED', manual_twist=Twist(), manual_deadline=.4,
        emergency_stop_active=False, control_mode='MANUAL', local_map_load_pending=False,
        get_parameter=lambda name: SimpleNamespace(value=(.4 if name=='manual_command_timeout' else .25)),
        trace_control_callback=lambda _: None,
        send_control_status=lambda *_args, **_kwargs: None,
        cmd_pub=SimpleNamespace(publish=sent.append))
    bridge._manual_timer = lambda: SwerveBridge._manual_timer(bridge)
    bridge._apply_manual_command = lambda data: SwerveBridge._apply_manual_command(bridge, data)
    SwerveBridge.manual_timer(bridge)
    assert sent and all(message.linear.x == .25 for message in sent)
    assert bridge.manual_deadline == pytest.approx(.79)
    assert mailbox.manual is None


@pytest.mark.parametrize('barrier', [
    {'type':'MANUAL_CMD','action':'STOP'}, {'type':'CONTROL_MODE','mode':'AUTONOMOUS'},
    {'type':'EMERGENCY_STOP'}, {'type':'CLEAR_EMERGENCY_STOP'}, {'type':'MANUAL_DISCONNECT'},
])
def test_lease_timer_cannot_consume_motion_ahead_of_pending_barrier(barrier):
    mailbox = ControlMailbox()
    mailbox.put(barrier)
    mailbox.put({'type': 'MANUAL_CMD', 'action': 'FORWARD'})
    assert mailbox.take_pending_manual() is None
    assert mailbox.get_nowait()['type'] == barrier['type']


@pytest.mark.parametrize('wheel_drift,passed', [(0.001, True), (0.20, False)])
def test_mechanical_settling_requires_actual_wheel_position_stability(monkeypatch, wheel_drift, passed):
    sys.path.insert(0, str(ROOT / 'scripts'))
    import end_to_end_acceptance as acceptance
    clock = [10.0]
    monkeypatch.setattr(acceptance.time, 'monotonic', lambda: clock[0])
    probe = acceptance.MotionProbe.__new__(acceptance.MotionProbe)
    probe.odom_velocity = probe.gazebo_velocity = (0.0, 0.0, 0.0)
    probe.gazebo_pose = (0.0, 0.0, 0.0)
    probe.sim_time = lambda: clock[0]
    probe.selected_cmd_events = [(10.0, 0.0, 0.0, 0.0)]
    positions = {'steer_front_joint': 0.0, 'steer_rear_joint': 0.0,
                 'wheel_front_drive_joint': 0.0, 'wheel_rear_drive_joint': 0.0}
    velocities = {'wheel_front_drive_joint': 0.20, 'wheel_rear_drive_joint': 0.0}
    def pump(*args):
        clock[0] += 0.1
        current = dict(positions, wheel_front_drive_joint=(clock[0] - 10) * wheel_drift)
        probe.joint_state = {'positions': current, 'velocities': velocities,
                             'monotonic_s': clock[0], 'source_sim_s': clock[0]}
        probe.selected_cmd_events = [(clock[0], 0.0, 0.0, 0.0)]
        probe.drive_events = [(clock[0], 0.0, 0.0)]
        probe.steering_events = [(clock[0], 0.0, 0.0)]
        probe.odom_sample_monotonic = clock[0]
        probe.gazebo_pose_sample_monotonic = clock[0]
    probe.pump = pump
    result = probe.wait_mechanical_settling(timeout=2.0)
    assert result['passed'] is passed
    if not passed:
        assert result['reason'] == 'MECHANICAL_SETTLING_SIM_TIMEOUT'
