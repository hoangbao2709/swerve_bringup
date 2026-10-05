import math

from swerve_bridge.navigation_map import transform_occupancy_grid, validate_target_coverage


def test_full_grid_is_resampled_into_registered_active_frame_without_changing_cells():
    transformed = transform_occupancy_grid(
        width=2, height=1, resolution=1.0, origin_x=0.0, origin_y=0.0,
        origin_yaw=0.0, data=[0, 100],
        transform={'tx': 0.0, 'ty': 0.0, 'yaw': math.pi / 2.0},
    )

    assert (transformed['width'], transformed['height']) == (1, 2)
    assert transformed['origin_x'] == -1.0
    assert transformed['origin_y'] == 0.0
    assert transformed['data'] == [0, 100]
    assert transformed['known_cells'] == 2
    assert transformed['unknown_cells'] == 0


def test_rotation_translation_bounds_cover_every_transformed_canonical_corner():
    transformed = transform_occupancy_grid(
        width=600, height=1200, resolution=0.05,
        origin_x=0.0, origin_y=0.0, origin_yaw=0.0,
        data=[0] * (600 * 1200),
        transform={'tx': -5.5, 'ty': 15.0, 'yaw': -math.pi / 2.0},
    )

    assert transformed['width'] == 1200
    assert transformed['height'] == 600
    assert transformed['min_x'] <= -5.5
    assert transformed['max_x'] >= 54.5
    assert transformed['min_y'] <= -15.0
    assert transformed['max_y'] >= 15.0


def test_target_coverage_distinguishes_outside_unknown_and_occupied_cells():
    grid = {
        'ready': True, 'width': 2, 'height': 2, 'resolution': 1.0,
        'origin_x': -1.0, 'origin_y': -1.0, 'origin_yaw': 0.0,
        'min_x': -1.0, 'max_x': 1.0, 'min_y': -1.0, 'max_y': 1.0,
        'navigation_map_id': 'NAV-22-SLAM-1',
        'navigation_map_revision': 'canonical-22-registration-4',
        'data': [0, 100, -1, 0],
    }

    outside = validate_target_coverage(grid, x=1.01, y=0.0)
    unknown = validate_target_coverage(grid, x=-0.5, y=0.5)
    occupied = validate_target_coverage(grid, x=0.5, y=-0.5)

    assert outside['code'] == 'TARGET_OUTSIDE_NAVIGATION_MAP'
    assert outside['navigation_map_id'] == 'NAV-22-SLAM-1'
    assert unknown['code'] == 'TARGET_IN_UNKNOWN_SPACE'
    assert occupied['code'] == 'TARGET_OCCUPIED'
    assert validate_target_coverage(grid, x=-0.5, y=-0.5) is None


def test_target_coverage_rejects_unavailable_or_malformed_navigation_map():
    assert validate_target_coverage(None, x=1.0, y=2.0)['code'] == 'NAVIGATION_MAP_NOT_READY'
    assert validate_target_coverage({'ready': True}, x=1.0, y=2.0)['code'] == 'NAVIGATION_MAP_INVALID'


def test_published_full_map_covers_distant_tags_that_exceed_the_old_live_slam_extent():
    registration = {'tx': -5.50943, 'ty': 14.99651, 'yaw': -1.57016}
    full_map = transform_occupancy_grid(
        width=600, height=1200, resolution=0.05,
        origin_x=0.0, origin_y=0.0, origin_yaw=0.0,
        data=[0] * (600 * 1200), transform=registration,
    )
    full_map.update({
        'ready': True, 'navigation_map_id': 'NAV-22-SLAM-session-1',
        'navigation_map_revision': 'session-session-1:canonical-22:registration-3',
    })

    # These are the backend-resolved Tag 1204/1205 poses from the accepted
    # registration pair; Tag 1204's X exceeds the old SLAM max X of 15.570609m.
    tag_1204 = (16.490, 0.014)
    tag_1205 = (21.990, 0.015)
    assert full_map['max_x'] > 15.570609
    assert validate_target_coverage(full_map, x=tag_1204[0], y=tag_1204[1]) is None
    assert validate_target_coverage(full_map, x=tag_1205[0], y=tag_1205[1]) is None

    old_slam_map = {
        'ready': True, 'width': 420, 'height': 597, 'resolution': 0.05,
        'origin_x': -5.429391, 'origin_y': -14.896761, 'origin_yaw': 0.0,
        'data': [0] * (420 * 597),
    }
    assert validate_target_coverage(old_slam_map, x=tag_1204[0], y=tag_1204[1])['code'] == (
        'TARGET_OUTSIDE_NAVIGATION_MAP')


def test_bridge_accepts_registration_for_authoritative_active_slam_map():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from swerve_bridge.bridge_node import SwerveBridge

    bridge = object.__new__(SwerveBridge)
    bridge.robot_id = 'R01'
    bridge.ros_map_revision = 22
    bridge.active_map_identity = lambda: {
        'active_map_id': 'SLAM-session-1',
        'active_map_revision': 'session-session-1',
    }
    bridge.active_map_source = lambda: 'SLAM_TOOLBOX'
    bridge.navigation_map_registration = None
    bridge.navigation_map_signature = None
    bridge.navigation_grid_occupancy = None
    bridge.navigation_map_status = {'ready': False}
    bridge.get_logger = lambda: SimpleNamespace(info=Mock(), error=Mock())
    bridge._publish_navigation_map_status = Mock()
    bridge.navigation_map_worker = SimpleNamespace(wake=Mock())

    bridge.apply_navigation_map_registration({
        'robot_id': 'R01',
        'canonical_map_revision': 22,
        'active_map_id': 'SLAM-session-1',
        'active_map_revision': 'session-session-1',
        'registration_revision': 3,
        'source': 'GAZEBO_CANONICAL_ALIGNMENT',
        'tx': 1.0, 'ty': 2.0, 'yaw': 0.5,
    })

    assert bridge.navigation_map_registration['registration_revision'] == 3
    assert bridge.navigation_map_status['ready'] is False
    assert bridge.navigation_map_status['reason'] == 'REGISTERED_NAVIGATION_MAP_REFRESHING'
    bridge.navigation_map_worker.wake.assert_called_once_with()
    bridge._publish_navigation_map_status.assert_called_once_with()
