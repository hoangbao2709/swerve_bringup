"""Regression tests for map-frame/Gazebo final-pose measurement."""
import math
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import end_to_end_acceptance as acceptance  # noqa: E402


def test_compose_planar_tf_links_with_parent_rotation():
    pose = acceptance.compose_planar_transforms(
        (10.0, 4.0, math.pi / 2),
        (2.0, 1.0, -math.pi / 2 + 0.02),
    )

    assert pose[0] == pytest.approx(9.0)
    assert pose[1] == pytest.approx(6.0)
    assert pose[2] == pytest.approx(0.02)


def test_latest_component_freshness_rejects_stale_and_future_tf():
    assert acceptance.transform_is_fresh(0.0)
    assert acceptance.transform_is_fresh(0.5)
    assert not acceptance.transform_is_fresh(0.5001)
    assert not acceptance.transform_is_fresh(-0.051)


def test_common_tf_time_interpolates_at_edge_intersection_without_extrapolation():
    # A newer map->odom sample does not make a latest-common-time lookup stale;
    # both edges are evaluated at the newest time available on both edges.
    assert acceptance.common_tf_sample_time(19.888, 19.797) == pytest.approx(19.797)
    assert acceptance.common_tf_sample_time(19.888, 19.797, 19.75) == pytest.approx(19.75)
    assert acceptance.common_tf_sample_time(19.888, 19.797, 19.798) is None
    assert acceptance.common_tf_sample_time(float('nan'), 19.797) is None


def test_controller_stop_observation_uses_live_simulation_ramp_and_rtf():
    required_sim_s = acceptance.controller_stop_sim_budget(3.7, 10.0, 50.0)
    assert required_sim_s == pytest.approx(0.41)
    assert acceptance.controller_stop_wall_budget(1.5, required_sim_s, 0.25) == pytest.approx(2.25)
    assert acceptance.controller_stop_wall_budget(1.5, required_sim_s, 0.8) == pytest.approx(1.5)


@pytest.mark.parametrize(('acceleration', 'rate'), [(0.0, 50.0), (10.0, 0.0), (float('nan'), 50.0)])
def test_controller_stop_measurement_rejects_invalid_runtime_parameters(acceleration, rate):
    with pytest.raises(ValueError):
        acceptance.controller_stop_sim_budget(3.7, acceleration, rate)


def test_pose_pair_requires_both_sources_after_localization_confirmation():
    assert acceptance.common_pose_pair_stamp(14.799, 14.762, 14.799) is None
    assert acceptance.common_pose_pair_stamp(14.820, 14.805, 14.799) == pytest.approx(14.805)
    assert acceptance.common_pose_pair_stamp(14.820, 14.805) == pytest.approx(14.805)
    assert acceptance.common_pose_pair_stamp(float('nan'), 14.805) is None


def test_navigation_map_requires_matching_active_identity_and_ready_revision():
    status = {
        'local_active_maps': {'R01': {
            'active_map_id': 'SLAM-session-a', 'active_map_revision': 'session-a',
        }},
        'navigation_maps': {'R01': {
            'ready': True, 'active_map_id': 'SLAM-session-a',
            'active_map_revision': 'session-a',
            'navigation_map_revision': 'session-a:canonical-23:registration-4',
            'canonical_map_revision': 23, 'registration_revision': 4,
            'registration_source': 'GAZEBO_CANONICAL_ALIGNMENT',
        }},
    }
    assert acceptance.ready_navigation_map_signature(status, 'R01') == (
        'SLAM-session-a', 'session-a', 'session-a:canonical-23:registration-4',
        '23', '4', 'GAZEBO_CANONICAL_ALIGNMENT')
    status['navigation_maps']['R01']['active_map_revision'] = 'stale'
    assert acceptance.ready_navigation_map_signature(status, 'R01') is None
    status['navigation_maps']['R01']['active_map_revision'] = 'session-a'
    status['navigation_maps']['R01']['ready'] = False
    assert acceptance.ready_navigation_map_signature(status, 'R01') is None


def test_navigation_mode_accepts_only_the_independently_verified_local_map_identity():
    status = {
        'runtime_state': 'NAVIGATION',
        'local_active_maps': {'R01': {
            'active_map_id': 'saved-map-1', 'active_map_revision': 'rev-1',
            'local_active_map_id': 'saved-map-1',
            'local_active_map_revision': 'rev-1',
            'map_content_revision': 'sha256:contents',
            'map_source': 'LOCAL_MAP', 'map_sync_status': 'LOCAL_ONLY',
        }},
        'navigation_maps': {'R01': {
            'ready': False, 'reason': 'WAITING_FOR_CANONICAL_MAP_AND_REGISTRATION',
        }},
        'robot_capabilities': {'R01': {'nav2_ready': True}},
    }
    selected = {'active_map_id': 'saved-map-1', 'active_map_revision': 'rev-1'}

    assert acceptance.ready_navigation_map_signature(
        status, 'R01', verified_local_map_identity=selected) == (
            'LOCAL_MAP', 'saved-map-1', 'rev-1', 'sha256:contents')
    assert acceptance.ready_navigation_map_signature(status, 'R01') is None

    status['local_active_maps']['R01']['active_map_revision'] = 'rev-stale'
    assert acceptance.ready_navigation_map_signature(
        status, 'R01', verified_local_map_identity=selected) is None
    status['local_active_maps']['R01']['active_map_revision'] = 'rev-1'
    status['robot_capabilities']['R01']['nav2_ready'] = False
    assert acceptance.ready_navigation_map_signature(
        status, 'R01', verified_local_map_identity=selected) is None
    status['robot_capabilities']['R01']['nav2_ready'] = True
    status['runtime_state'] = 'UNIFIED'
    assert acceptance.ready_navigation_map_signature(
        status, 'R01', verified_local_map_identity=selected) is None


def test_gazebo_ground_truth_is_projected_using_measured_map_alignment():
    reference_map = (0.0, 0.0, 0.0)
    reference_world = (15.0, 5.5, math.pi / 2)
    current_world = (14.0, 6.5, math.pi / 2)

    pose = acceptance.map_pose_from_world(
        reference_map, reference_world, current_world)

    assert pose[0] == pytest.approx(1.0)
    assert pose[1] == pytest.approx(1.0)
    assert pose[2] == pytest.approx(0.0)


def test_only_settled_pose_confirmation_allows_bounded_stationary_tf_age():
    sample = {
        'frame_ids_valid': True,
        'component_time_skew_sim_s': 0.1,
        'pose': (1.0, 2.0, 0.3),
        'map_to_odom': {'age_sim_s': 0.6},
        'odom_to_base': {'age_sim_s': 0.7},
        'fresh': False,
    }

    assert not acceptance.transform_is_fresh(0.6)
    assert acceptance.transform_is_confirmable_when_settled(sample)
    sample['odom_to_base']['age_sim_s'] = 1.0001
    assert not acceptance.transform_is_confirmable_when_settled(sample)
    sample['odom_to_base']['age_sim_s'] = 0.1
    sample['component_time_skew_sim_s'] = 0.5001
    assert not acceptance.transform_is_confirmable_when_settled(sample)


def test_map_to_gazebo_pair_skew_uses_simulation_stamps():
    sample = {'stamp_sim_s': 4.75}
    assert acceptance.pose_pair_skew_sim_s(sample, 5.0) == pytest.approx(0.25)
    assert acceptance.pose_pair_skew_sim_s(sample, None) is None


def test_acceptance_failure_diagnostics_name_the_failed_pose_contracts():
    checks = {
        'action_succeeded': True,
        'projected_gazebo_goal_translation_within_0_05m': False,
        'map_tf_gazebo_yaw_agreement_within_0_05rad': False,
        'terminal_tf_fresh': True,
    }

    assert acceptance.failed_acceptance_conditions(checks) == [
        'projected_gazebo_goal_translation_within_0_05m',
        'map_tf_gazebo_yaw_agreement_within_0_05rad',
    ]
    assert acceptance.NAV_GOAL_XY_TOLERANCE_M == 0.05
    assert acceptance.NAV_GOAL_YAW_TOLERANCE_RAD == 0.05


class _ModeStatusProbe:
    def __init__(self, statuses):
        self.control_statuses = list(statuses)

    def wait_until(self, predicate, _timeout, ws):
        for status in ws.statuses:
            self.control_statuses.append(status)
            if predicate():
                return True
        return predicate()


class _ModeStatusSocket:
    def __init__(self, statuses):
        self.statuses = statuses
        self.sent = []

    def send(self, payload):
        self.sent.append(json.loads(payload))


def test_mode_transition_requires_requested_and_applied_status_for_same_fresh_id():
    old = {'robot_id': 'R01', 'mode': 'AUTONOMOUS', 'applied_mode': 'AUTONOMOUS',
           'accepted': True, 'mode_transition_state': 'APPLIED', 'request_id': 'old'}
    probe = _ModeStatusProbe([old])
    ws = _ModeStatusSocket([
        old,
        {'robot_id': 'R01', 'mode': 'AUTONOMOUS', 'requested_mode': 'AUTONOMOUS',
         'accepted': True, 'mode_transition_state': 'REQUESTED', 'request_id': 'new'},
        {'robot_id': 'R01', 'mode': 'AUTONOMOUS', 'applied_mode': 'AUTONOMOUS',
         'accepted': True, 'mode_transition_state': 'APPLIED', 'request_id': 'new'},
    ])

    passed, reason = acceptance.set_mode(probe, ws, 'R01', 'AUTONOMOUS')

    assert passed is True
    assert reason is None
    assert ws.sent == [{'type': 'ROBOT_MODE', 'robot_id': 'R01', 'mode': 'AUTONOMOUS'}]


def test_mode_transition_does_not_accept_unpaired_delayed_applied_status():
    probe = _ModeStatusProbe([])
    ws = _ModeStatusSocket([
        {'robot_id': 'R01', 'mode': 'AUTONOMOUS', 'applied_mode': 'AUTONOMOUS',
         'accepted': True, 'mode_transition_state': 'APPLIED', 'request_id': 'stale'},
    ])

    passed, reason = acceptance.set_mode(probe, ws, 'R01', 'AUTONOMOUS')

    assert passed is False
    assert reason == 'no_correlated_ROBOT_CONTROL_STATUS_APPLIED'


def test_gazebo_pose_is_interpolated_at_the_tf_stamp_without_extrapolation():
    samples = [
        {'simulation_time_s': 2.0, 'pose': (0.0, 1.0, math.radians(179))},
        {'simulation_time_s': 2.2, 'pose': (1.0, 3.0, math.radians(-179))},
    ]

    aligned = acceptance.interpolate_gazebo_pose(samples, 2.1)

    assert aligned['simulation_time_s'] == pytest.approx(2.1)
    assert aligned['pose'][0] == pytest.approx(0.5)
    assert aligned['pose'][1] == pytest.approx(2.0)
    assert abs(abs(aligned['pose'][2]) - math.pi) < 1e-6
    assert aligned['source_stamps_sim_s'] == [2.0, 2.2]
    assert acceptance.interpolate_gazebo_pose(samples, 1.99) is None
    assert acceptance.interpolate_gazebo_pose(samples, 2.21) is None


def test_out_of_order_gazebo_callbacks_are_kept_in_source_time_order():
    samples = []
    for stamp, x in ((10.0, 1.0), (12.0, 3.0), (11.0, 2.0)):
        samples = acceptance.insert_gazebo_pose_sample(samples, {
            'simulation_time_s': stamp,
            'pose': (x, 0.0, 0.0),
        })

    assert [row['simulation_time_s'] for row in samples] == [10.0, 11.0, 12.0]
    assert samples[-1]['pose'][0] == 3.0
    aligned = acceptance.interpolate_gazebo_pose(samples, 11.5)
    assert aligned['pose'][0] == pytest.approx(2.5)


def test_duplicate_gazebo_source_stamp_replaces_without_reordering():
    samples = [
        {'simulation_time_s': 10.0, 'pose': (1.0, 0.0, 0.0)},
        {'simulation_time_s': 12.0, 'pose': (3.0, 0.0, 0.0)},
    ]
    samples = acceptance.insert_gazebo_pose_sample(samples, {
        'simulation_time_s': 10.0 + 5e-10,
        'pose': (1.5, 0.0, 0.0),
    })

    assert len(samples) == 2
    assert [row['simulation_time_s'] for row in samples] == [10.0, 12.0]
    assert samples[0]['pose'][0] == 1.5
