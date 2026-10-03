import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

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


def _nav_safety_bridge():
    bridge = object.__new__(SwerveBridge)
    bridge.robot_id = 'R01'
    bridge.control_mode = 'AUTONOMOUS'
    bridge.emergency_stop_active = False
    bridge.goal_request_pending = False
    bridge.active_goal = None
    bridge.active_pose_goal = None
    bridge.active_context = None
    bridge.active_pose_context = None
    bridge.paused_pose_context = None
    bridge.cancel_pending = False
    bridge.pending_cancel_state = None
    bridge.pending_replan = None
    bridge.nav_state = 'NAVIGATING'
    bridge.cmd_pub = SimpleNamespace(publish=Mock())
    bridge.estop_pub = SimpleNamespace(publish=Mock())
    bridge.send = Mock()
    bridge.send_nav_status = Mock()
    bridge.now = lambda: '2026-10-03T00:00:00Z'
    return bridge


def _accepted_pose_handle():
    cancel_future = SimpleNamespace(add_done_callback=Mock())
    result_future = SimpleNamespace(add_done_callback=Mock())
    handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=Mock(return_value=cancel_future),
        get_result_async=Mock(return_value=result_future),
    )
    return handle, cancel_future, result_future


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


@pytest.mark.parametrize(('source_type', 'source_id'), [('MAP_POINT', None), ('TAG', '1301')])
def test_common_pose_goal_cancel_waits_for_a_pending_nav2_acceptance(source_type, source_id):
    bridge = _nav_safety_bridge()
    bridge.goal_request_pending = True
    context = {
        'robot_id': 'R01', 'frame_id': 'map', 'x': 2.5, 'y': 3.5, 'yaw': 1.2,
        'source_type': source_type, 'source_id': source_id,
    }

    bridge.cancel_navigation({'type': 'NAV_CANCEL', 'robot_id': 'R01'})
    assert bridge.goal_request_pending
    assert bridge.pending_cancel_state == 'CANCELLED'
    assert bridge.cmd_pub.publish.call_count == 1

    handle, _cancel_future, result_future = _accepted_pose_handle()
    bridge.pose_goal_response(SimpleNamespace(result=lambda: handle), context)
    handle.cancel_goal_async.assert_called_once_with()
    result_future.add_done_callback.assert_called_once()
    assert bridge.cancel_pending
    assert bridge.nav_state == 'CANCELLED'
    assert not any(message.get('goal', {}).get('status') == 'ACTIVE'
                   for message in (call.args[0] for call in bridge.send.call_args_list))


def test_estop_during_pending_nav2_acceptance_cancels_goal_and_blocks_clear_until_terminal():
    bridge = _nav_safety_bridge()
    bridge.goal_request_pending = True
    context = {'robot_id': 'R01', 'frame_id': 'map', 'x': 2.5, 'y': 3.5, 'yaw': 1.2}

    bridge.emergency_stop({'stop_id': 'estop-1'})
    assert bridge.emergency_stop_active
    assert bridge.pending_cancel_state == 'EMERGENCY_STOPPED'
    rejected = bridge.clear_emergency_stop({})
    assert rejected['code'] == 'CLEAR_ESTOP_REJECTED_GOAL_PENDING'
    assert rejected['emergency_stop_active'] is True
    assert rejected['pre_stop_navigation_terminal'] is False
    assert bridge.estop_pub.publish.call_count == 1

    handle, cancel_future, result_future = _accepted_pose_handle()
    bridge.pose_goal_response(SimpleNamespace(result=lambda: handle), context)
    handle.cancel_goal_async.assert_called_once_with()
    assert bridge.cancel_pending
    rejected = bridge.clear_emergency_stop({})
    assert rejected['code'] == 'CLEAR_ESTOP_REJECTED_GOAL_PENDING'
    assert bridge.emergency_stop_active
    assert bridge.estop_pub.publish.call_count == 1

    cancel_callback = cancel_future.add_done_callback.call_args.args[0]
    cancel_callback(SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[1])))
    assert bridge.cancel_pending  # Request acceptance is not Nav2 goal termination.
    bridge.pose_goal_result(
        SimpleNamespace(result=lambda: SimpleNamespace(status=bridge_node.GoalStatus.STATUS_CANCELED)),
        handle, context,
    )
    assert bridge.active_pose_goal is None
    assert not bridge.cancel_pending
    assert bridge.emergency_stop_active
    applied = bridge.clear_emergency_stop({})
    assert applied['code'] == 'CLEAR_ESTOP_APPLIED'
    assert applied['emergency_stop_active'] is False
    assert applied['pre_stop_navigation_terminal'] is True
    assert not bridge.emergency_stop_active
    assert bridge.pending_cancel_state is None
    assert bridge.estop_pub.publish.call_count == 2


def test_estop_on_active_nav2_goal_does_not_clear_before_cancel_result():
    bridge = _nav_safety_bridge()
    context = {'robot_id': 'R01', 'frame_id': 'map', 'x': 2.5, 'y': 3.5, 'yaw': 1.2}
    handle, cancel_future, _result_future = _accepted_pose_handle()
    bridge.active_pose_goal = handle
    bridge.active_pose_context = context

    bridge.emergency_stop({'stop_id': 'estop-active'})
    handle.cancel_goal_async.assert_called_once_with()
    rejected = bridge.clear_emergency_stop({})
    assert rejected['code'] == 'CLEAR_ESTOP_REJECTED_GOAL_PENDING'
    callback = cancel_future.add_done_callback.call_args.args[0]
    callback(SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[1])))
    assert bridge.cancel_pending
    assert bridge.emergency_stop_active
    bridge.pose_goal_result(
        SimpleNamespace(result=lambda: SimpleNamespace(status=bridge_node.GoalStatus.STATUS_CANCELED)),
        handle, context,
    )
    assert bridge.active_pose_goal is None
    assert not bridge.cancel_pending
    applied = bridge.clear_emergency_stop({})
    assert applied['code'] == 'CLEAR_ESTOP_APPLIED'
    assert not bridge.emergency_stop_active
    assert handle.cancel_goal_async.call_count == 1


def test_clear_estop_local_control_returns_correlated_applied_or_pending_result():
    bridge = _nav_safety_bridge()
    bridge.emergency_stop_active = True
    bridge.goal_request_pending = True
    request = {'operation': 'CLEAR_ESTOP', 'request_id': 'clear-pending'}

    bridge.local_control(request)
    rejected = bridge.send.call_args.args[0]
    assert rejected['type'] == 'LOCAL_CONTROL_RESULT'
    assert rejected['request_id'] == 'clear-pending'
    assert rejected['ok'] is False
    assert rejected['result']['code'] == 'CLEAR_ESTOP_REJECTED_GOAL_PENDING'
    assert rejected['result']['emergency_stop_active'] is True

    bridge.send.reset_mock()
    bridge.goal_request_pending = False
    applied = {'operation': 'CLEAR_ESTOP', 'request_id': 'clear-terminal'}
    bridge.local_control(applied)
    result = bridge.send.call_args.args[0]
    assert result['type'] == 'LOCAL_CONTROL_RESULT'
    assert result['request_id'] == 'clear-terminal'
    assert result['ok'] is True
    assert result['result']['code'] == 'CLEAR_ESTOP_APPLIED'
    assert result['result']['emergency_stop_active'] is False
    assert result['result']['pre_stop_navigation_terminal'] is True
