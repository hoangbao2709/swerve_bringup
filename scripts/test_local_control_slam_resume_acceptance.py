"""Focused regressions for resumed SLAM acceptance readiness evidence."""

from __future__ import annotations

import base64
from collections import deque
from pathlib import Path
import sys
from threading import RLock
import zlib

import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

from local_control_slam_resume_acceptance import (
    ResumeMotionProbe,
    ResumeReadinessGate,
    map_extension_evidence,
    resumed_map_save_gate,
    resume_tf_qos_profiles,
)
import end_to_end_acceptance as acceptance


def _map_summary(width=2, height=1, known_cells=1, signature='a' * 64,
                 origin=(0.0, 0.0, 0.0)):
    return {
        'width': width, 'height': height, 'resolution': 0.05,
        'known_cells': known_cells, 'signature': signature,
        'origin': list(origin[:2]),
    }


def _map_stats(rows, now=100.0, sim_now=10.0):
    gate = ResumeReadinessGate.__new__(ResumeReadinessGate)
    gate.lock = RLock()
    gate.samples = {'map': deque(rows)}
    return gate._topic_stats(
        'map', now, sim_now,
        max_wall_age=gate.MAX_MAP_WALL_AGE_S,
        max_sim_age=gate.MAX_MAP_SIM_AGE_S,
        min_rate_hz=0.0,
        require_progress=False,
    )


def _snapshot(width, height, values):
    payload = bytes(int(value) + 1 for value in values)
    return {
        'data_encoding': 'zlib-base64-offset1',
        'width': width, 'height': height, 'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
        'data_zlib_base64': base64.b64encode(zlib.compress(payload)).decode('ascii'),
    }


def test_stationary_restored_map_does_not_need_stamp_or_content_progress():
    stats = _map_stats([(100.0, 10.0)])
    liveness = ResumeReadinessGate._map_live_evidence(
        stats, _map_summary(), slam_active=True)

    assert stats['rate_hz'] is None
    assert stats['stamp_increasing'] is False
    assert stats['fresh'] is True
    assert liveness['passed'] is True
    assert liveness['required_rate_hz'] is None
    assert liveness['stamp_increasing_required'] is False


def test_repeated_static_map_stamps_pass_only_while_wall_and_sim_fresh():
    stats = _map_stats([(99.0, 10.0), (100.0, 10.0)])
    assert stats['stamp_increasing'] is False
    assert stats['fresh'] is True
    assert ResumeReadinessGate._map_live_evidence(
        stats, _map_summary(), slam_active=True)['passed'] is True

    stale_stats = _map_stats([(100.0, 10.0)], now=104.0, sim_now=14.0)
    stale = ResumeReadinessGate._map_live_evidence(
        stale_stats, _map_summary(), slam_active=True)
    assert stale['passed'] is False
    assert 'map_message_fresh' in stale['failed_conditions']


def test_empty_map_sample_array_is_a_waiting_gate_not_an_exception():
    stats = _map_stats([])
    liveness = ResumeReadinessGate._map_live_evidence(
        stats, _map_summary(), slam_active=True)
    assert liveness['passed'] is False
    assert 'map_message_fresh' in liveness['failed_conditions']
    assert 'map_received_within_wall_bound' in liveness['failed_conditions']


def test_dynamic_tf_uses_bounded_wall_and_sim_age_not_one_second_wall_cutoff():
    gate = ResumeReadinessGate.__new__(ResumeReadinessGate)
    gate.lock = RLock()
    gate.tf_edges = {
        ('odom', 'base_footprint'): {
            'stamp_s': 10.094, 'wall_received_monotonic_s': 103.57,
            'static': False,
        },
    }
    gate.tf_edge_samples = {
        ('odom', 'base_footprint'): deque([(103.57, 10.094)]),
    }

    stats = gate._transform_stats(('odom', 'base_footprint'), 10.4, 105.0)

    assert stats['wall_age_s'] > 1.0
    assert stats['wall_age_s'] < 3.5
    assert stats['sim_age_s'] == pytest.approx(0.306)
    assert stats['fresh'] is True
    assert stats['max_wall_age_s'] == 3.5


def test_dynamic_tf_fails_closed_after_wall_freshness_bound():
    gate = ResumeReadinessGate.__new__(ResumeReadinessGate)
    gate.lock = RLock()
    gate.tf_edges = {
        ('odom', 'base_footprint'): {
            'stamp_s': 10.094, 'wall_received_monotonic_s': 101.0,
            'static': False,
        },
    }
    gate.tf_edge_samples = {('odom', 'base_footprint'): deque([(101.0, 10.094)])}

    stats = gate._transform_stats(('odom', 'base_footprint'), 10.4, 105.0)

    assert stats['wall_age_s'] == 4.0
    assert stats['sim_age_s'] == pytest.approx(0.306)
    assert stats['fresh'] is False


def test_tf_listener_uses_latest_sample_qos_for_dynamic_edges():
    dynamic, static = resume_tf_qos_profiles()

    assert dynamic.depth == 1
    assert dynamic.reliability == ReliabilityPolicy.BEST_EFFORT
    assert dynamic.durability == DurabilityPolicy.VOLATILE
    assert static.depth == 100
    assert static.reliability == ReliabilityPolicy.RELIABLE
    assert static.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_sim_stamped_sensor_wall_bound_tracks_rtf_but_stays_capped():
    gate = ResumeReadinessGate.__new__(ResumeReadinessGate)
    gate.lock = RLock()
    rows = [(97.8, 99.7), (99.6, 99.85), (100.0, 100.0)]
    gate.samples = {'scan': deque(rows)}

    fresh = gate._topic_stats(
        'scan', now=101.668, sim_now=100.205,
        sample_rows=rows, gazebo_rtf=0.13)
    stale = gate._topic_stats(
        'scan', now=103.6, sim_now=100.205,
        sample_rows=rows, gazebo_rtf=0.13)

    assert fresh['wall_age_s'] == pytest.approx(1.668)
    assert fresh['max_wall_age_s'] == pytest.approx(2.077, abs=0.01)
    assert fresh['rate_hz'] < 1.0
    assert fresh['sim_rate_hz'] > 1.0
    assert fresh['required_rate_hz'] == 1.0
    assert fresh['rate_requirement_met_by'] == 'simulation_time'
    assert fresh['fresh'] is True
    assert stale['max_wall_age_s'] <= gate.MAX_DYNAMIC_TOPIC_WALL_AGE_S
    assert stale['fresh'] is False


def test_gazebo_model_state_freshness_uses_sim_rate_at_low_rtf():
    gate = ResumeReadinessGate.__new__(ResumeReadinessGate)
    gate.lock = RLock()
    rows = [
        (95.07 + i * (4.9 / 18.0), 99.677 + i * (0.323 / 18.0))
        for i in range(19)
    ]
    gate.samples = {'model_states': deque(rows)}

    stats = gate._topic_stats(
        'model_states', now=100.0, sim_now=100.0,
        sample_rows=rows, gazebo_rtf=0.0657)

    assert stats['rate_hz'] < gate.MIN_RATE_HZ_BY_TOPIC['model_states']
    assert stats['sim_rate_hz'] > gate.MIN_RATE_HZ_BY_TOPIC['model_states']
    assert stats['fresh'] is True


def test_scan_tf_accepts_only_exact_or_tightly_bounded_latest_lookup():
    exact = {'fresh': False, 'stamp_s': None}
    latest = {'fresh': True, 'stamp_s': 100.007, 'sim_age_s': 0.2}
    delayed = ResumeReadinessGate._scan_tf_readiness(
        True, 100.0, exact, latest, odom_base_fresh=True, map_odom_fresh=True)
    assert delayed['passed'] is True
    assert delayed['exact_latest_scan_lookup_passed'] is False
    assert delayed['bounded_latest_transform_fallback_passed'] is True

    too_far = ResumeReadinessGate._scan_tf_readiness(
        True, 100.0, exact, {**latest, 'stamp_s': 100.051},
        odom_base_fresh=True, map_odom_fresh=True)
    stale_edge = ResumeReadinessGate._scan_tf_readiness(
        True, 100.0, exact, latest,
        odom_base_fresh=False, map_odom_fresh=True)
    assert too_far['passed'] is False
    assert stale_edge['passed'] is False


def test_empty_model_pose_or_twist_array_waits_without_indexerror():
    probe = ResumeMotionProbe.__new__(ResumeMotionProbe)
    probe.malformed_model_state_samples = 0
    message = ModelStates()
    message.name = [acceptance.EXPECTED_ROBOT_ENTITY]
    message.pose = [Pose()]
    message.twist = []

    probe._model_states_cb(message)

    assert probe.malformed_model_state_samples == 1


def test_map_extension_detects_newly_known_cell_and_rejects_short_origin():
    before = _snapshot(2, 1, [100, -1])
    after = _snapshot(2, 1, [100, 0])
    extension = map_extension_evidence(before, after)
    assert extension['passed'] is True
    assert extension['newly_known_cells'] == 1

    before['origin'] = [0.0]
    malformed = map_extension_evidence(before, after)
    assert malformed['passed'] is False
    assert 'invalid OccupancyGrid geometry' in malformed['reason']


def test_unchanged_map_cannot_pass_resumed_save_or_extension():
    before = _snapshot(2, 1, [100, -1])
    after = _snapshot(2, 1, [100, -1])
    extension = map_extension_evidence(before, after)
    save_checks = {'distinct_id': True, 'artifacts_valid': True}

    assert extension['passed'] is False
    assert extension['newly_known_cells'] == 0
    assert extension['known_cells_outside_prior_extent'] == 0
    assert resumed_map_save_gate(extension, save_checks) is False

    assert resumed_map_save_gate({'passed': True}, save_checks) is True
