from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from end_to_end_acceptance import MotionProbe
from save_load_map_acceptance import (compare_active_grid_with_saved_map,
                                      initial_pose_confirmation,
                                      initial_pose_after_supervised_load,
                                      transition_status)


def test_transition_status_handles_null_or_absent_optional_transition():
    assert transition_status({'transition': None}) is None
    assert transition_status({}) is None
    assert transition_status({'transition': {'status': 'ROLLING_BACK'}}) == 'ROLLING_BACK'


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
