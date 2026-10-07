import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from swerve_bridge import bridge_node
from swerve_bridge.bridge_node import SwerveBridge


def _compute_path_goal():
    return SimpleNamespace(
        start=SimpleNamespace(
            header=SimpleNamespace(frame_id='', stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ),
        ),
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


def _lifecycle_nodes(runtime_state='NAVIGATION'):
    map_node = 'canonical_map_server' if runtime_state == 'UNIFIED' else 'map_server'
    return (map_node, 'controller_server', 'planner_server', 'behavior_server',
            'bt_navigator', 'waypoint_follower')


def _nav_safety_bridge():
    bridge = object.__new__(SwerveBridge)
    bridge.robot_id = 'R01'
    bridge.runtime_state = 'NAVIGATION'
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
    bridge.pending_cancel_reason = None
    bridge.pending_replan = None
    bridge.tag_route_context = None
    bridge.tag_route_waiting_auth = None
    bridge.paused_pose_context = None
    bridge.nav_state = 'NAVIGATING'
    bridge.cmd_pub = SimpleNamespace(publish=Mock())
    bridge.estop_pub = SimpleNamespace(publish=Mock())
    bridge._logger = SimpleNamespace(info=Mock(), warning=Mock(), error=Mock())
    bridge.send = Mock()
    bridge.send_nav_status = Mock()
    bridge.now = lambda: '2026-10-03T00:00:00Z'
    return bridge


def _tag_route_context():
    return {
        'robot_id': 'R01', 'frame_id': 'map', 'source_type': 'TAG',
        'route_revision': 'route-revision-1', 'graph_revision': 'graph-revision-1',
        'route_nodes': [11], 'route_tag_revisions': {'11': 'tag-revision-11'},
        'route_points': [
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'kind': 'START', 'tag_id': None},
            {'x': 2.0, 'y': 0.0, 'yaw': 0.0, 'kind': 'TAG', 'tag_id': 11},
        ],
        'active_map_id': 'SLAM-session-1', 'active_map_revision': 'session-rev-1',
        'navigation_map_revision': 'nav-rev-1', 'registration_revision': 3,
        'route_index': 1,
    }


def test_tag_route_leg_waits_for_backend_authorization_before_nav2_planning(monkeypatch):
    monkeypatch.setattr(bridge_node, 'ComputePathToPose', SimpleNamespace(Goal=_compute_path_goal))
    bridge = _nav_safety_bridge()
    context = _tag_route_context()
    preview_action = Mock()
    preview_action.send_goal_async.return_value = SimpleNamespace(add_done_callback=Mock())
    bridge.runtime_state = 'NAVIGATION'
    bridge.navigation_grid_occupancy = None
    bridge._tag_route_authorized_now = Mock(return_value=(True, None))
    bridge._lookup_robot_pose = Mock(return_value=({'x': 1.0, 'y': 0.5, 'yaw': math.pi / 2}, 'map'))
    bridge.path_preview_client = SimpleNamespace(send_goal_async=preview_action.send_goal_async)
    bridge.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))

    bridge.tag_route_context = dict(context)
    bridge._request_tag_route_leg(context)

    auth_request = bridge.send.call_args.args[0]
    assert auth_request['type'] == 'TAG_ROUTE_LEG_AUTH_REQUEST'
    assert auth_request['route_revision'] == context['route_revision']
    assert auth_request['route_index'] == 1
    assert auth_request['route_node_id'] == 11
    preview_action.send_goal_async.assert_not_called()

    bridge.accept_tag_route_leg_auth({
        'auth_request_id': auth_request['auth_request_id'],
        'route_revision': context['route_revision'], 'route_index': 1, 'approved': True,
    })
    goal = preview_action.send_goal_async.call_args.args[0]
    assert goal.start.header.frame_id == 'map'
    assert (goal.start.pose.position.x, goal.start.pose.position.y) == (1.0, 0.5)
    assert (goal.goal.pose.position.x, goal.goal.pose.position.y) == (2.0, 0.0)
    assert goal.use_start is True


def test_tag_route_leg_auth_timeout_stops_route_and_rejects_late_approval(monkeypatch):
    bridge = _nav_safety_bridge()
    context = _tag_route_context()
    preview_action = Mock()
    bridge.runtime_state = 'NAVIGATION'
    bridge.navigation_grid_occupancy = None
    bridge._tag_route_authorized_now = Mock(return_value=(True, None))
    bridge._lookup_robot_pose = Mock(return_value=({'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'map'))
    bridge.path_preview_client = SimpleNamespace(send_goal_async=preview_action.send_goal_async)
    bridge.get_parameter = lambda _name: SimpleNamespace(value=0.5)
    bridge.get_logger = lambda: SimpleNamespace(info=Mock(), warning=Mock())
    bridge.tag_route_context = dict(context)

    bridge._request_tag_route_leg(context)
    auth_request = bridge.send.call_args.args[0]
    waiting = bridge.tag_route_waiting_auth
    waiting['requested_monotonic'] = time.monotonic() - 1.0
    bridge._expire_tag_route_leg_auth()
    bridge.accept_tag_route_leg_auth({
        'auth_request_id': auth_request['auth_request_id'],
        'route_revision': context['route_revision'], 'route_index': 1, 'approved': True,
    })

    preview_action.send_goal_async.assert_not_called()
    bridge.cmd_pub.publish.assert_called()
    assert bridge.tag_route_context is None
    assert bridge.tag_route_waiting_auth is None
    assert bridge.send_nav_status.call_args.args[1] == 'FAILED'
    assert 'TAG_ROUTE_AUTH_TIMEOUT' in bridge.send_nav_status.call_args.args[2]


def test_cancel_while_waiting_for_tag_route_auth_prevents_late_nav2_dispatch():
    bridge = _nav_safety_bridge()
    context = _tag_route_context()
    bridge.tag_route_context = dict(context)
    bridge.tag_route_waiting_auth = {
        'auth_request_id': 'request-1', 'route_revision': context['route_revision'],
        'route_index': 1, 'context': context, 'requested_monotonic': time.monotonic(),
    }
    bridge._dispatch_tag_route_leg = Mock()

    bridge.cancel_navigation({'type': 'NAV_CANCEL'})
    bridge.accept_tag_route_leg_auth({
        'auth_request_id': 'request-1', 'route_revision': context['route_revision'],
        'route_index': 1, 'approved': True,
    })

    bridge._dispatch_tag_route_leg.assert_not_called()
    bridge.cmd_pub.publish.assert_called_once()
    assert bridge.tag_route_context is None
    assert bridge.tag_route_waiting_auth is None
    assert bridge.send_nav_status.call_args.args[1] == 'CANCELLED'


def _accepted_pose_handle():
    cancel_future = SimpleNamespace(add_done_callback=Mock())
    result_future = SimpleNamespace(add_done_callback=Mock())
    handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=Mock(return_value=cancel_future),
        get_result_async=Mock(return_value=result_future),
    )
    return handle, cancel_future, result_future


def test_nav2_readiness_requires_both_path_and_navigation_action_servers():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'NAVIGATION'
    bridge.nav2_lifecycle_nodes = _lifecycle_nodes()
    bridge.nav2_lifecycle_states = {name: 'active' for name in bridge.nav2_lifecycle_nodes}
    bridge.path_preview_client = SimpleNamespace(server_is_ready=lambda: True)
    bridge.nav_pose_client = SimpleNamespace(server_is_ready=lambda: True)
    assert bridge.nav2_action_servers_ready()

    bridge.path_preview_client = SimpleNamespace(server_is_ready=lambda: False)
    assert not bridge.nav2_action_servers_ready()

    bridge.path_preview_client = SimpleNamespace(server_is_ready=lambda: True)
    bridge.nav_pose_client = SimpleNamespace(server_is_ready=lambda: False)
    assert not bridge.nav2_action_servers_ready()


def test_nav2_readiness_fails_closed_when_bt_navigator_is_inactive():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'NAVIGATION'
    bridge.nav2_lifecycle_nodes = _lifecycle_nodes()
    bridge.nav2_lifecycle_states = {name: 'active' for name in bridge.nav2_lifecycle_nodes}
    bridge.nav2_lifecycle_states['bt_navigator'] = 'inactive'
    bridge.path_preview_client = SimpleNamespace(server_is_ready=lambda: True)
    bridge.nav_pose_client = SimpleNamespace(server_is_ready=lambda: True)

    status = bridge.nav2_lifecycle_status()
    assert status['ready'] is False
    assert status['blocker_code'] == 'NAV2_LIFECYCLE_NOT_ACTIVE'
    assert '/bt_navigator=inactive' in status['blocker_reason']
    assert bridge.nav2_action_servers_ready() is False


def test_point_goal_is_not_dispatched_while_bt_navigator_is_inactive():
    bridge = object.__new__(SwerveBridge)
    bridge.robot_id = 'R01'
    bridge.runtime_state = 'NAVIGATION'
    bridge.control_mode = 'AUTONOMOUS'
    bridge.emergency_stop_active = False
    bridge.loaded_local_map_id = None
    bridge.loaded_local_map_revision = None
    bridge.local_map_load_pending = False
    bridge.active_goal = None
    bridge.active_pose_goal = None
    bridge.goal_request_pending = False
    bridge.nav2_lifecycle_nodes = _lifecycle_nodes()
    bridge.nav2_lifecycle_states = {name: 'active' for name in bridge.nav2_lifecycle_nodes}
    bridge.nav2_lifecycle_states['bt_navigator'] = 'inactive'
    action = Mock()
    bridge.nav_pose_client = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=action.send_goal_async)
    bridge.send_nav_status = Mock()

    bridge.navigate_pose({'x': 2.0, 'y': 1.0, 'yaw': 0.0, 'frame_id': 'map'})

    action.send_goal_async.assert_not_called()
    assert bridge.send_nav_status.call_args.args[1] == 'FAILED'
    assert 'NAV2_LIFECYCLE_NOT_ACTIVE' in bridge.send_nav_status.call_args.args[2]
    assert '/bt_navigator=inactive' in bridge.send_nav_status.call_args.args[2]


def test_active_map_point_uses_shared_compute_path_and_navigate_to_pose_actions(monkeypatch):
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
    bridge.runtime_state = 'NAVIGATION'
    bridge.nav2_lifecycle_nodes = _lifecycle_nodes()
    bridge.nav2_lifecycle_states = {name: 'active' for name in bridge.nav2_lifecycle_nodes}
    bridge.control_mode = 'AUTONOMOUS'
    bridge.emergency_stop_active = False
    bridge.active_map_identity = lambda: map_identity.copy()
    bridge.navigation_map_status = {}
    bridge.navigation_grid_occupancy = None
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
        'source_type': 'ACTIVE_MAP_POINT',
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
        'source_type': 'ACTIVE_MAP_POINT',
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


def test_unified_preview_requires_registered_full_map_and_binds_its_revision(monkeypatch):
    monkeypatch.setattr(bridge_node, 'ComputePathToPose', SimpleNamespace(Goal=_compute_path_goal))
    active = {
        'active_map_id': 'SLAM-session-1',
        'active_map_revision': 'session-session-1',
        'canonical_map_revision': 22,
    }
    registration = {
        'tx': -5.5, 'ty': 15.0, 'yaw': -math.pi / 2.0,
    }
    nav_map = {
        'ready': True,
        'navigation_map_id': 'NAV-22-SLAM-session-1',
        'navigation_map_revision': 'session-session-1:canonical-22:registration-4',
        'resolution': 0.05, 'width': 1200, 'height': 600,
        'origin_x': -5.5, 'origin_y': -15.0, 'origin_yaw': 0.0,
        'min_x': -5.5, 'max_x': 54.5, 'min_y': -15.0, 'max_y': 15.0,
        'data': [0] * (1200 * 600),
    }
    action = Mock()
    action.send_goal_async.return_value = SimpleNamespace(add_done_callback=Mock())
    bridge = object.__new__(SwerveBridge)
    bridge.robot_id = 'R01'
    bridge.runtime_state = 'UNIFIED'
    bridge.nav2_lifecycle_nodes = _lifecycle_nodes('UNIFIED')
    bridge.nav2_lifecycle_states = {name: 'active' for name in bridge.nav2_lifecycle_nodes}
    bridge.control_mode = 'AUTONOMOUS'
    bridge.emergency_stop_active = False
    bridge.active_map_identity = lambda: active.copy()
    bridge.navigation_map_status = {
        'ready': True,
        'navigation_map_id': nav_map['navigation_map_id'],
        'navigation_map_revision': nav_map['navigation_map_revision'],
    }
    bridge.navigation_grid_occupancy = nav_map
    bridge._map_to_base = lambda x, y, _stamp: (x, y, 0.0)
    bridge.path_preview_client = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=action.send_goal_async)
    bridge.path_preview_goals = {}
    bridge.path_preview_approvals = {}
    bridge.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))
    bridge.send = Mock()

    target = {'x': 16.49, 'y': 0.014, 'yaw': 0.0}
    base = {
        'type': 'PATH_PREVIEW', 'request_id': 'unified-tag-1204', **target,
        'frame_id': 'map', 'source_type': 'ACTIVE_MAP_POINT',
        'active_map_id': active['active_map_id'],
        'active_map_revision': active['active_map_revision'],
    }
    bridge.preview_path({**base, 'navigation_map_revision': 'stale-registration'})
    rejected = bridge.send.call_args.args[0]
    assert rejected['reason_code'] == 'NAVIGATION_MAP_REVISION_MISMATCH'
    assert action.send_goal_async.call_count == 0

    bridge.preview_path({**base, 'x': 60.0,
                         'navigation_map_revision': nav_map['navigation_map_revision']})
    outside = bridge.send.call_args.args[0]
    assert outside['reason_code'] == 'TARGET_OUTSIDE_NAVIGATION_MAP'
    assert action.send_goal_async.call_count == 0

    bridge.preview_path({**base, 'navigation_map_revision': nav_map['navigation_map_revision']})
    goal = action.send_goal_async.call_args.args[0]
    assert goal.goal.header.frame_id == 'map'
    assert goal.goal.pose.position.x == target['x']
    assert bridge.path_preview_goals['unified-tag-1204'] == target

    approval = {
        'goal': target.copy(), 'active_map_id': active['active_map_id'],
        'active_map_revision': active['active_map_revision'],
        'navigation_map_revision': nav_map['navigation_map_revision'],
        'created_monotonic': time.monotonic(),
    }
    bridge.path_preview_approvals['unified-tag-1204'] = approval
    nav_goal_data = {
        **base, 'preview_request_id': 'unified-tag-1204',
        'navigation_map_revision': nav_map['navigation_map_revision'],
    }
    assert bridge.consume_path_preview(nav_goal_data)

    bridge.path_preview_approvals['unified-tag-1204'] = approval
    bridge.navigation_map_status['navigation_map_revision'] = 'new-registration'
    assert not bridge.consume_path_preview(nav_goal_data)


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


def test_tag_route_yaw_only_motion_cannot_complete_a_translating_waypoint():
    bridge = _nav_safety_bridge()
    route = _tag_route_context()
    bridge.tag_route_context = dict(route)
    handle, _cancel_future, _result_future = _accepted_pose_handle()
    bridge.active_pose_goal = handle
    context = {**route, 'route_index': 1, 'route_node_id': 11}
    # Simulate a controller that achieved the requested heading but never
    # translated toward a waypoint two metres away.
    bridge._lookup_robot_pose = Mock(return_value=({'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'map'))
    bridge.get_parameter = lambda name: SimpleNamespace(
        value=0.20 if name == 'tag_route_xy_tolerance_m' else math.radians(10.0))
    bridge.get_logger = lambda: SimpleNamespace(warning=Mock(), info=Mock())

    bridge.pose_goal_result(
        SimpleNamespace(result=lambda: SimpleNamespace(
            status=bridge_node.GoalStatus.STATUS_SUCCEEDED)), handle, context)

    status_call = bridge.send_nav_status.call_args
    assert status_call.args[1] == 'FAILED'
    assert 'TAG_ROUTE_NODE_NOT_REACHED' in status_call.args[2]
    assert 'xy_error=2.000m' in status_call.args[2]
    assert bridge.tag_route_context is None
    assert not any(call.args[1] == 'TAG_ROUTE_NODE_REACHED'
                   for call in bridge.send_nav_status.call_args_list)


def test_registration_drift_cancel_reason_survives_nav2_terminal_result():
    bridge = _nav_safety_bridge()
    handle, _cancel_future, _result_future = _accepted_pose_handle()
    context = {'robot_id': 'R01', 'frame_id': 'map', 'preview_request_id': 'preview-1'}
    bridge.active_pose_goal = handle
    bridge.active_pose_context = context

    bridge.cancel_navigation({
        'type': 'CANCEL_NAVIGATION',
        'reason_code': 'MAP_REGISTRATION_DRIFT',
        'reason': 'alignment changed during the approved action',
    })
    assert bridge.pending_cancel_state == 'CANCELLED'
    assert bridge.pending_cancel_reason.startswith('MAP_REGISTRATION_DRIFT:')
    handle.cancel_goal_async.assert_called_once_with()

    bridge.pose_goal_result(
        SimpleNamespace(result=lambda: SimpleNamespace(
            status=bridge_node.GoalStatus.STATUS_CANCELED)), handle, context)

    status_call = bridge.send_nav_status.call_args
    assert status_call.args[0] == context
    assert status_call.args[1] == 'CANCELLED'
    assert 'MAP_REGISTRATION_DRIFT:' in status_call.args[2]
    assert bridge.pending_cancel_reason is None


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
