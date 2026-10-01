import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from mechanical_settling import MechanicalSettling


@pytest.mark.parametrize('rolling_rate,passed', [(0.0003, True), (0.0004, False), (0.0011, False)])
def test_measured_geometry_never_relaxes_existing_wheel_limit(rolling_rate, passed):
    radius = .0675
    monitor = MechanicalSettling(0., 0., timeout_sim=2., wheel_radius=radius)
    for i in range(22):
        sim = i * .1
        result = monitor.update(sim, sim, sample(sim, wheel=sim * rolling_rate / radius,
                                                 body=sim * rolling_rate))
        if result is not None:
            break
    assert result['passed'] is passed
    assert result['wheel_position_rate_limit'] == pytest.approx(.005)
    assert result['wheel_rolling_drift_rates'][0] == pytest.approx(rolling_rate)


def test_invalid_geometry_cannot_disable_rotation_checks():
    with pytest.raises(ValueError):
        MechanicalSettling(0., 0., wheel_radius=float('nan'))


def test_measured_geometry_can_only_tighten_wheel_limit():
    monitor = MechanicalSettling(0., 0., wheel_radius=1.)
    assert monitor.wheel_position_rate == pytest.approx(.001)


def sample(wall, *, wheel=0.0, body=0.0, steering=0.0, drive=0.0):
    return {'selected': [0., 0., 0.], 'drive': [drive, drive],
            'body_velocity': [0., 0., 0.], 'odom_velocity': [0., 0.007, 0.007],
            'wheel_positions': [wheel, wheel], 'wheel_velocities': [-0.154, -0.107],
            'steering_positions': [steering, steering], 'steering_targets': [steering, steering],
            'body_pose': [body, 0., 0.], 'source_wall_times': [wall] * 6}


def test_low_rtf_uses_simulation_window_not_wall_timeout():
    monitor = MechanicalSettling(0., 0.)
    result = None
    for i in range(51):
        wall, sim = float(i), i * .02
        result = monitor.update(sim, wall, sample(wall, wheel=sim * .001))
        if result is not None:
            break
    assert result['passed']
    assert result['settling_sim_seconds'] >= .8
    assert result['gazebo_rtf'] == pytest.approx(.02)


@pytest.mark.parametrize('kind', ['wheel', 'body', 'steering', 'drive'])
def test_actual_rotation_slip_body_drift_and_target_chatter_do_not_settle(kind):
    monitor = MechanicalSettling(0., 0., timeout_sim=2.)
    for i in range(22):
        sim = i * .1
        values = {kind: sim * .1 if kind in ('wheel', 'body') else (1.0 if i % 2 else 0.0)}
        result = monitor.update(sim, sim, sample(sim, **values))
        if result is not None:
            break
    assert not result['passed']
    assert result['reason'] == 'MECHANICAL_SETTLING_SIM_TIMEOUT'


def test_clock_stall_and_rewind_are_explicit():
    monitor = MechanicalSettling(1., 0.)
    assert monitor.update(1., 5., sample(5.))['reason'] == 'SIM_CLOCK_STALLED'
    monitor = MechanicalSettling(1., 0.)
    assert monitor.update(.9, .1, sample(.1))['reason'] == 'SIM_CLOCK_REWOUND'


def test_wall_watchdog_is_not_a_physics_timeout():
    monitor = MechanicalSettling(0., 0., timeout_wall=3.)
    for i in range(4):
        result = monitor.update(i * .01, float(i), None)
    assert result['reason'] == 'MECHANICAL_SETTLING_WALL_WATCHDOG'


def test_oscillating_wheel_position_is_not_mistaken_for_zero_net_rotation():
    monitor = MechanicalSettling(0., 0., timeout_sim=2.)
    for i in range(22):
        sim = i * .1
        result = monitor.update(sim, sim, sample(sim, wheel=.05 if i % 2 else -.05))
        if result:
            break
    assert not result['passed']


@pytest.mark.parametrize('invalid', ['nonfinite', 'stale'])
def test_invalid_or_stale_feedback_cannot_become_a_settled_window(invalid):
    monitor = MechanicalSettling(0., 0., timeout_sim=2.)
    for i in range(22):
        sim = i * .1
        row = sample(sim)
        if invalid == 'nonfinite':
            row['wheel_positions'][0] = float('nan')
        else:
            row['source_wall_times'][0] = sim - 3.
        result = monitor.update(sim, sim, row)
        if result:
            break
    assert not result['passed']


@pytest.mark.parametrize('missing', ['wheel_positions', 'drive', 'steering_targets'])
def test_missing_nonfinite_or_stale_feedback_cannot_pass(missing):
    monitor = MechanicalSettling(0., 0., timeout_sim=2.)
    for i in range(22):
        sim = i * .1
        row = sample(sim)
        del row[missing]
        result = monitor.update(sim, sim, row)
        if result:
            break
    assert result['reason'] == 'MECHANICAL_SETTLING_SIM_TIMEOUT'
