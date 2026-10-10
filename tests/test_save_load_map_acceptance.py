from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from end_to_end_acceptance import MotionProbe
from save_load_map_acceptance import (compare_active_grid_with_saved_map,
                                      initial_pose_confirmation,
                                      initial_pose_after_supervised_load,
                                      odom_tf_freshness,
                                      prelocalization_map_ready,
                                      wait_command_arbiter_mode,
                                      transition_status)


def test_transition_status_handles_null_or_absent_optional_transition():
    assert transition_status({'transition': None}) is None
    assert transition_status({}) is None
    assert transition_status({'transition': {'status': 'ROLLING_BACK'}}) == 'ROLLING_BACK'


def test_saved_map_can_be_ready_before_amcl_initial_pose_without_nav2_ready():
    ready, evidence = prelocalization_map_ready(
        active_identity=True, fresh_map=True, grid_matches=True,
        nav2_available=True, map_server_state={'id': 3, 'label': 'active'})

    assert ready is True
    assert all(evidence['checks'].values())
    assert evidence['nav2_ready_expected_before_initial_pose'] is False


@pytest.mark.parametrize('overrides,failed_check', [
    ({'active_identity': False}, 'active_map_identity_and_revision'),
    ({'fresh_map': False}, 'fresh_navigation_map_message'),
    ({'grid_matches': False}, 'saved_occupancy_exactly_matches'),
    ({'nav2_available': False}, 'navigation_runtime_available'),
    ({'map_server_state': {'id': 1, 'label': 'unconfigured'}},
     'map_server_lifecycle_active'),
])
def test_saved_map_prelocalization_gate_fails_closed(overrides, failed_check):
    arguments = {
        'active_identity': True, 'fresh_map': True, 'grid_matches': True,
        'nav2_available': True, 'map_server_state': {'id': 3, 'label': 'active'},
    }
    arguments.update(overrides)

    ready, evidence = prelocalization_map_ready(**arguments)

    assert ready is False
    assert evidence['checks'][failed_check] is False


def test_post_reset_odom_tf_is_required_before_initial_pose_not_map_tf():
    sample = {
        'parent_frame': 'odom', 'child_frame': 'base_footprint',
        'age_sim_s': 0.12, 'simulation_clock_age_wall_s': 0.08,
        'tf_epoch_generation': 5,
    }

    freshness = odom_tf_freshness(sample, expected_epoch=5, current_epoch=5)

    assert freshness['passed'] is True


@pytest.mark.parametrize('overrides,failed_check', [
    ({'age_sim_s': 0.51}, 'dynamic_transform_fresh_in_sim_time'),
    ({'parent_frame': 'map'}, 'odom_to_base_footprint_frames'),
    ({'tf_epoch_generation': 4}, 'current_tf_epoch'),
    ({'simulation_clock_age_wall_s': 1.01}, 'simulation_clock_fresh_in_wall_time'),
])
def test_post_reset_odom_tf_rejects_stale_wrong_or_previous_epoch(overrides, failed_check):
    sample = {
        'parent_frame': 'odom', 'child_frame': 'base_footprint',
        'age_sim_s': 0.12, 'simulation_clock_age_wall_s': 0.08,
        'tf_epoch_generation': 5,
    }
    sample.update(overrides)

    freshness = odom_tf_freshness(sample, expected_epoch=5, current_epoch=5)

    assert freshness['passed'] is False
    assert freshness['checks'][failed_check] is False


def test_saved_nav2_map_comparison_checks_loaded_occupancy_and_unknown_cells(tmp_path):
    yaml_path = tmp_path / 'saved.yaml'
    image_path = tmp_path / 'saved.pgm'
    yaml_path.write_text(
        'image: saved.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [1.0, 2.0, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
    image_path.write_bytes(b'P5\n# comment\n2 2\n255\n' + bytes((0, 205, 254, 205)))
    origin = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0),
                             orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    info = SimpleNamespace(width=2, height=2, resolution=0.05, origin=origin)
    grid = SimpleNamespace(info=info, data=[0, -1, 100, -1])

    result = compare_active_grid_with_saved_map(grid, yaml_path, image_path)

    assert result['matches'] is True
    assert result['expected_unknown_cells'] == 2
    assert result['actual_unknown_cells'] == 2
    grid.data[1] = 0
    mismatch = compare_active_grid_with_saved_map(grid, yaml_path, image_path)
    assert mismatch['matches'] is False
    assert mismatch['mismatch_count'] == 1


def test_motion_probe_tracks_nav2_map_separately_from_slam_map():
    probe = SimpleNamespace(
        navigation_map=None,
        navigation_map_sample_count=0,
        navigation_map_sample_monotonic=None,
    )
    grid = SimpleNamespace(info=SimpleNamespace(width=2, height=2),
                           data=[0, 0, 100, -1])

    MotionProbe._navigation_map_cb(probe, grid)

    assert probe.navigation_map is grid
    assert probe.navigation_map_sample_count == 1
    assert probe.navigation_map_sample_monotonic is not None


def test_saved_map_handoff_waits_for_fresh_applied_manual_arbiter_diagnostics():
    class Probe:
        command_diagnostics = [
            (1.0, {'robot_id': 'R01', 'active_control_mode': 'AUTONOMOUS',
                   'estop_active': False}),
            (2.0, {'robot_id': 'R01', 'active_control_mode': 'MANUAL',
                   'estop_active': False, 'active_command_source': 'NONE'}),
        ]

        def wait_until(self, predicate, _timeout, _ws):
            return predicate()

    ready, evidence = wait_command_arbiter_mode(
        Probe(), None, 'R01', 'MANUAL', after_count=1)

    assert ready
    assert evidence['active_control_mode'] == 'MANUAL'
    assert evidence['estop_active'] is False

    Probe.command_diagnostics = [
        (1.0, {'robot_id': 'R01', 'active_control_mode': 'MANUAL',
               'estop_active': False}),
        (2.0, {'robot_id': 'R01', 'active_control_mode': 'AUTONOMOUS',
               'estop_active': False}),
    ]
    ready, _ = wait_command_arbiter_mode(Probe(), None, 'R01', 'MANUAL', after_count=0)
    assert not ready

    ready, _ = wait_command_arbiter_mode(Probe(), None, 'R01', 'MANUAL', after_count=2)
    assert not ready


def test_respawn_initial_pose_uses_saved_map_time_alignment_not_stale_startup_pose():
    initial, method = initial_pose_after_supervised_load(
        (0.0, 0.0, 0.0), (15.0, 5.5, 1.57079632679),
        (14.0, 6.5, 1.57079632679))

    assert method == 'measured_pre_load_map_to_gazebo_transform'
    assert initial == pytest.approx((1.0, 1.0, 0.0))


def test_unchanged_simulated_robot_keeps_fresh_saved_map_pose():
    initial, method = initial_pose_after_supervised_load(
        (1.2, 0.8, 0.4), (14.0, 6.5, 1.57), (14.0, 6.5, 1.57))

    assert method == 'same_simulated_robot_pose_before_and_after_transition'
    assert initial == pytest.approx((1.2, 0.8, 0.4))


def test_initial_pose_confirmation_rejects_a_fresh_but_displaced_transform():
    request = (0.0, 0.0, 0.0)
    sample = {
        'fresh': True,
        'stamp_sim_s': 10.1,
        'tf_epoch_generation': 4,
        'pose': (0.15, 0.0, 0.0),
    }

    confirmation = initial_pose_confirmation(sample, request, 10.0, 4, 4)

    assert confirmation['passed'] is False
    assert confirmation['position_error_m'] == pytest.approx(0.15)
    assert confirmation['failed_checks'] == ['position_error_within_0_05m']


def test_initial_pose_confirmation_requires_post_request_fresh_tf_in_same_epoch():
    sample = {
        'fresh': True,
        'stamp_sim_s': 10.1,
        'tf_epoch_generation': 4,
        'pose': (0.01, -0.01, 0.03),
    }

    confirmation = initial_pose_confirmation(sample, (0.0, 0.0, 0.0), 10.0, 4, 4)
    assert confirmation['passed'] is True
    assert confirmation['failed_checks'] == []

    wrong_epoch = initial_pose_confirmation(sample, (0.0, 0.0, 0.0), 10.0, 3, 4)
    assert wrong_epoch['passed'] is False
    assert 'current_tf_epoch' in wrong_epoch['failed_checks']
