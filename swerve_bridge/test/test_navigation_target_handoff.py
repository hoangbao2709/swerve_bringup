import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

from swerve_bridge import bridge_node
from swerve_bridge.bridge_node import SwerveBridge


def _compute_path_goal():
    return SimpleNamespace(
        goal=SimpleNamespace(
            header=SimpleNamespace(frame_id='', stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ),
        ),
        use_start=True,
    )


def _navigate_to_pose_goal():
    return SimpleNamespace(
        pose=SimpleNamespace(
            header=SimpleNamespace(frame_id='', stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ),
        ),
    )


def test_tag_resolved_pose_uses_shared_compute_path_and_navigate_to_pose_actions(monkeypatch):
    map_identity = {
        'active_map_id': 'CANONICAL',
        'active_map_revision': '21',
        'canonical_map_revision': 21,
    }
    preview_action = Mock()
    preview_future = SimpleNamespace(add_done_callback=Mock())
    preview_action.send_goal_async.return_value = preview_future
    nav_action = Mock()
    nav_future = SimpleNamespace(add_done_callback=Mock())
    nav_action.send_goal_async.return_value = nav_future
    monkeypatch.setattr(bridge_node, 'ComputePathToPose', SimpleNamespace(Goal=_compute_path_goal))
    monkeypatch.setattr(bridge_node, 'NavigateToPose', SimpleNamespace(Goal=_navigate_to_pose_goal))

    bridge = object.__new__(SwerveBridge)
    bridge.robot_id = 'R01'
    bridge.control_mode = 'AUTONOMOUS'
    bridge.emergency_stop_active = False
    bridge.active_map_identity = lambda: map_identity.copy()
    bridge.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))
    bridge.path_preview_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=preview_action.send_goal_async,
    )
    bridge.path_preview_goals = {}
    bridge.path_preview_approvals = {}
    bridge.send = Mock()

    target = {'x': 2.5, 'y': 3.5, 'yaw': 1.2}
    request_id = 'tag-preview-1301'
    bridge.preview_path({
        'type': 'PATH_PREVIEW', 'request_id': request_id, **target, 'frame_id': 'map',
        'source_type': 'TAG', 'source_id': '1301', 'tag_id': 1301,
        'active_map_id': 'CANONICAL', 'active_map_revision': '21',
    })

    preview_goal = preview_action.send_goal_async.call_args.args[0]
    assert preview_goal.goal.header.frame_id == 'map'
    assert preview_goal.goal.pose.position.x == target['x']
    assert preview_goal.goal.pose.position.y == target['y']
    assert preview_goal.goal.pose.orientation.z == math.sin(target['yaw'] / 2.0)
    assert preview_goal.goal.pose.orientation.w == math.cos(target['yaw'] / 2.0)
    assert preview_goal.use_start is False
    assert bridge.send.call_count == 0  # ComputePathToPose preview alone does not navigate.

    bridge.path_preview_approvals[request_id] = {
        'goal': target.copy(), 'active_map_id': 'CANONICAL', 'active_map_revision': '21',
        'created_monotonic': time.monotonic(),
    }
    nav_data = {
        'type': 'NAV_GOAL', **target, 'frame_id': 'map', 'preview_request_id': request_id,
        'source_type': 'TAG', 'source_id': '1301', 'tag_id': 1301,
        'active_map_id': 'CANONICAL', 'active_map_revision': '21',
    }
    assert not bridge.consume_path_preview({**nav_data, 'x': target['x'] + 0.01})
    assert bridge.consume_path_preview(nav_data)

    bridge.loaded_local_map_id = None
    bridge.loaded_local_map_revision = None
    bridge.local_map_load_pending = False
    bridge.active_goal = None
    bridge.active_pose_goal = None
    bridge.goal_request_pending = False
    bridge.nav_state = 'IDLE'
    bridge.nav_pose_client = SimpleNamespace(server_is_ready=lambda: True,
                                             send_goal_async=nav_action.send_goal_async)
    bridge.navigate(nav_data)

    nav_goal = nav_action.send_goal_async.call_args.args[0]
    assert nav_goal.pose.header.frame_id == 'map'
    assert nav_goal.pose.pose.position.x == target['x']
    assert nav_goal.pose.pose.position.y == target['y']
    assert nav_goal.pose.pose.orientation.z == math.sin(target['yaw'] / 2.0)
    assert nav_goal.pose.pose.orientation.w == math.cos(target['yaw'] / 2.0)
