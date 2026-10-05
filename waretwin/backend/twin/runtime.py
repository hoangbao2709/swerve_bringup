from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from asgiref.sync import sync_to_async
from channels.layers import get_channel_layer
from django.conf import settings
from pydantic import TypeAdapter, ValidationError

from .ai import copilot as copilot_ai
from .schema import ClientMessage, TwinState
from .sim.engine import SIM, SimEngine
from .sim.navgrid import load_layout
from .sim.whatif import run_whatif
from .conveyor_plc import PLCSimulator
from .coordinates import ros_pose_to_waretwin, ros_twist_to_waretwin, validated_canonical_pose
from .control_timing import profile_async, profile_sync

log = logging.getLogger(__name__)
TICK_S = SIM['TICK_S']
HEATMAP_EVERY = 30
client_adapter: TypeAdapter[Any] = TypeAdapter(ClientMessage)
MAPPING_CAPABLE_MODES = frozenset({'MAPPING', 'UNIFIED'})
NAVIGATION_CAPABLE_MODES = frozenset({'NAVIGATION', 'UNIFIED'})


def _persist_events(run_id: str, events: list[dict[str, Any]]) -> None:
    if not events:
        return
    from .models import EventLog
    rows = []
    for e in events:
        rows.append(EventLog(
            run_id=run_id,
            event_id=str(e.get('id', '')),
            tick=int(e.get('tick', 0)),
            type=str(e.get('type', 'UNKNOWN')),
            source=str(e.get('source', '')),
            severity=str(e.get('severity', 'INFO')),
            message=str(e.get('message', '')),
            robot_id=e.get('robot_id'), task_id=e.get('task_id'), zone_id=e.get('zone_id'),
            conveyor_id=e.get('conveyor_id'), camera_id=e.get('camera_id'),
            payload=e.get('payload') or {},
        ))
    EventLog.objects.bulk_create(rows, batch_size=200)


class TwinRuntime:
    """Authoritative runtime adapter for local simulation or ROS telemetry.

    The frontend contract remains FULL/PATCH/HEATMAP plus persistent REST state.
    ``LOCAL_SIM`` advances ``SimEngine``; ``GAZEBO_ROS`` and ``REAL_ROBOT`` only
    update robot state from the authenticated bridge and never invent movement.
    """

    SECTIONS = ('tasks', 'lifts', 'zones', 'conveyors', 'cameras', 'sensors', 'people', 'alerts')

    def __init__(self) -> None:
        self.layout_dir = Path(settings.BASE_DIR) / 'layouts'
        self.layout_dir.mkdir(exist_ok=True)
        active = self.layout_dir / 'active.json'
        if active.exists():
            try:
                self.layout = json.loads(active.read_text(encoding='utf-8'))
            except Exception as exc:
                log.warning(
                    'Unable to load active layout %s; falling back to packaged layout: %s',
                    active,
                    type(exc).__name__,
                    exc_info=True,
                )
                self.layout = load_layout()
        else:
            self.layout = load_layout()
        self.seed = int(os.getenv('TWIN_SEED', '42'))
        self.engine = SimEngine(self.layout, seed=self.seed)
        self.engine.external_scheduler = True
        self.plc = PLCSimulator(self.layout)
        self.engine.state['conveyors'] = self.plc.snapshot()
        self.runtime_mode = str(getattr(settings, 'WARETWIN_RUNTIME_MODE', 'GAZEBO_ROS'))
        self.robot_gateway = None
        self._prepare_external_cache()
        self.run_id = uuid.uuid4().hex[:8]
        self.speed: int = 1
        self.paused = os.getenv('TWIN_AUTOPLAY', '1') != '1'
        self.client_count = 0
        self._prev: dict[str, Any] = {}
        self._prev_subsys = ''
        self._sent_decision = ''
        self.last_sent_tick = 0
        self.tick_rate_actual = 0.0
        self.last_progress = time.monotonic()
        self.loop_errors = 0
        self.last_error: str | None = None
        self._task: asyncio.Task | None = None
        self._whatif_lock = asyncio.Lock()
        self.channel_layer = None
        self.ros_bridge_connected = False
        self.connected_robot_ids: set[str] = set()
        self.robot_bridge_heartbeats: dict[str, float] = {}
        self.robot_pose_heartbeats: dict[str, float] = {}
        self.bridge_status = 'DISCONNECTED' if self.is_external else 'LOCAL'
        self.last_telemetry_at: float | None = None
        self.last_telemetry_iso: str | None = None
        self.last_ros_heartbeat: float | None = None
        self.nav2_state = 'OFFLINE' if self.runtime_mode != 'LOCAL_SIM' else 'LOCAL'
        self.operation_mode = 'IDLE' if self.is_external else 'SIMULATION'
        self.ros_diagnostics: dict[str, Any] = {
            'ros': False, 'gazebo': False, 'controller_manager': False,
            'slam': False, 'nav2': False, 'tf': False, 'lidar': False,
            'nodes': [], 'topics': [], 'controllers': [],
            'simulation_time': None, 'last_update_at': None,
        }
        self.ros_diagnostics_received_monotonic: float | None = None
        self.published_map_revision: int | None = None
        self.published_map_version: int = 0
        self.ros_map_revision: int | None = None
        self.gazebo_map_revision: int | None = None
        self.nav2_map_revision: int | None = None
        self.tag_map_revision: int | None = None
        self.map_tf_status = False
        self.map_sync_error: str | None = None
        self.robot_map_sync: dict[str, dict[str, Any]] = {}
        self.robot_map_snapshots: dict[str, dict[str, Any]] = {}
        self.robot_runtime_map_snapshots: dict[str, dict[str, Any]] = {}
        self.robot_slam_map_snapshots: dict[str, dict[str, Any]] = {}
        self.visualization_clients = set()
        self.local_map_overrides: dict[str, str] = {}
        self.local_map_revisions: dict[str, str] = {}
        self.local_map_status: dict[str, dict[str, Any]] = {}
        self.robot_map_geometry: dict[str, dict[str, Any]] = {}
        self.local_map_transitions: set[str] = set()
        self.pending_local_map_loads: dict[str, dict[str, Any]] = {}
        self.pending_slam_session_resumes: dict[str, dict[str, Any]] = {}
        self.robot_mapping_state: dict[str, str] = {}
        self.robot_mapping_elapsed_s: dict[str, float] = {}
        self.robot_mapping_sessions: dict[str, str] = {}
        self.robot_runtime_modes: dict[str, str] = {}
        self.robot_command_diagnostics: dict[str, dict[str, Any]] = {}
        self.path_preview_requests: dict[tuple[str, str], dict[str, Any]] = {}
        self.approved_path_previews: dict[tuple[str, str], dict[str, Any]] = {}
        self.path_preview_results: dict[tuple[str, str], dict[str, Any]] = {}
        self.expired_path_previews: dict[tuple[str, str], float] = {}
        self.path_preview_invalidations: dict[tuple[str, str], str] = {}
        self.command_ownership: dict[str, dict[str, Any]] = {}
        self.robot_lidar_streams: dict[str, dict[str, Any]] = {}
        self._snapshot_prev()

    @property
    def is_external(self) -> bool:
        return self.runtime_mode in ('GAZEBO_ROS', 'REAL_ROBOT')

    def _prepare_external_cache(self) -> None:
        if not self.is_external:
            return
        for robot in self.engine.state.get('robots', {}).values():
            robot['status'] = 'OFFLINE'
            robot['fsm'] = 'OFFLINE'
            robot['navigation_state'] = 'OFFLINE'
            robot['last_telemetry_at'] = None
            robot['battery_reported'] = False
            robot['vx'] = robot['vy'] = robot['wz'] = 0.0
            robot['path'] = []
            robot['path_index'] = 0

    def gateway(self):
        if self.robot_gateway is None:
            from .gateways.ros_bridge import RosBridgeGateway
            self.robot_gateway = RosBridgeGateway()
        return self.robot_gateway

    def online_robot_ids(self) -> list[str]:
        """Return robot IDs with a live authenticated bridge and recent heartbeat."""
        timeout = float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
        now = time.monotonic()
        return sorted(
            robot_id for robot_id in self.connected_robot_ids
            if now - self.robot_bridge_heartbeats.get(robot_id, 0.0) <= timeout
        )

    def robot_bridge_online(self, robot_id: str) -> bool:
        return str(robot_id).strip() in self.online_robot_ids()

    def active_map_state(self, robot_id: str) -> dict[str, Any]:
        """Describe the map currently used by one robot without conflating it
        with the fleet's canonical map revision.
        """
        rid = str(robot_id or '').strip()
        canonical_revision = self.published_map_revision
        local_id = self.local_map_overrides.get(rid)
        if local_id:
            status = self.local_map_status.get(rid, {})
            revision = self.local_map_revisions.get(rid)
            ready = bool(status.get('loaded') and status.get('map_id') == local_id and revision)
            return {
                'active_map_id': local_id if ready else None,
                'active_map_revision': revision if ready else None,
                'local_active_map_id': local_id,
                'local_active_map_revision': revision,
                'canonical_map_revision': canonical_revision,
                'map_source': 'LOCAL_MAP',
                'map_sync_status': 'LOCAL_ONLY' if ready else 'LOADING',
            }

        robot_mode = str(self.robot_runtime_modes.get(rid) or self.operation_mode).upper()
        if robot_mode in MAPPING_CAPABLE_MODES:
            session_id = str(self.robot_mapping_sessions.get(rid) or '')
            snapshot = self.robot_slam_map_snapshots.get(rid, {})
            map_data = snapshot.get('map') if isinstance(snapshot, dict) else {}
            map_data = map_data if isinstance(map_data, dict) else {}
            snapshot_session = str(map_data.get('mapping_session_id') or '')
            map_id = (f'SLAM-{session_id}' if session_id else
                      str(map_data.get('active_map_id') or ''))
            revision = str(map_data.get('active_map_revision') or '')
            ready = bool(session_id and snapshot_session == session_id and map_id and revision)
            return {
                'active_map_id': map_id or None,
                'active_map_revision': revision or None,
                'local_active_map_id': None,
                'local_active_map_revision': None,
                'canonical_map_revision': canonical_revision,
                'map_source': 'SLAM_TOOLBOX',
                'map_sync_status': 'LOCAL_ONLY' if ready else 'LOADING',
            }

        row = self.robot_map_sync.get(rid, {})
        ros_revision = row.get('ros_revision', self.ros_map_revision)
        if row.get('status'):
            status = str(row['status'])
        else:
            from .map_sync import map_sync_status
            require_nav2, require_tag_map = self.map_sync_requirements()
            status = map_sync_status(
                published_revision=canonical_revision,
                ros_revision=ros_revision,
                gazebo_revision=row.get('gazebo_revision', self.gazebo_map_revision),
                nav2_revision=row.get('nav2_revision', self.nav2_map_revision),
                tag_map_revision=row.get('tag_map_revision', self.tag_map_revision),
                tf_status=bool(row.get('tf_status', self.map_tf_status)),
                require_nav2=require_nav2,
                require_tag_map=require_tag_map,
                ros_connected=self.robot_bridge_online(rid),
                error=row.get('error', self.map_sync_error),
                external=self.is_external,
            )
        if (status == 'SYNCED' and canonical_revision is not None
                and ros_revision == canonical_revision):
            status = 'CANONICAL'
        return {
            'active_map_id': 'CANONICAL' if ros_revision is not None else None,
            'active_map_revision': str(ros_revision) if ros_revision is not None else None,
            'local_active_map_id': None,
            'local_active_map_revision': None,
            'canonical_map_revision': canonical_revision,
            'map_sync_status': status,
            'map_source': 'CANONICAL' if ros_revision is not None else None,
        }

    def robot_capabilities(self, robot_id: str) -> dict[str, bool]:
        """Compute capabilities from runtime mode plus measured bridge state."""
        rid = str(robot_id or '').strip()
        mode = str(self.robot_runtime_modes.get(rid) or self.operation_mode).upper()
        diagnostics = self.ros_diagnostics
        mapping_diag = diagnostics.get('mapping') if isinstance(diagnostics.get('mapping'), dict) else {}
        command_state = self.robot_command_diagnostics.get(rid, {})
        command_fresh = bool(
            command_state.get('received_monotonic') is not None
            and time.monotonic() - float(command_state['received_monotonic'])
            <= float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
        )
        diagnostics_fresh = bool(
            self.ros_diagnostics_received_monotonic is not None
            and time.monotonic() - self.ros_diagnostics_received_monotonic
            <= float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
        )
        mapping_available = bool(
            self.is_external and self.robot_bridge_online(rid)
            and diagnostics_fresh and mode in MAPPING_CAPABLE_MODES and diagnostics.get('slam')
        )
        mapping_active = bool(
            mapping_available and str(mapping_diag.get('slam_state') or '').upper() == 'ACTIVE'
            and mapping_diag.get('map_live') is True
            and str(self.robot_mapping_state.get(rid) or 'MAPPING').upper() == 'MAPPING'
        )
        nav2_available = bool(
            self.is_external and self.robot_bridge_online(rid)
            and diagnostics_fresh and mode in NAVIGATION_CAPABLE_MODES and diagnostics.get('nav2')
        )
        nav2_ready = bool(nav2_available and diagnostics.get('nav2_ready') is True)
        active_map = self.active_map_state(rid)
        map_ready = bool(
            active_map.get('active_map_id') and active_map.get('active_map_revision')
            and active_map.get('map_sync_status') in ('CANONICAL', 'LOCAL_ONLY')
            and (active_map.get('map_source') != 'SLAM_TOOLBOX' or mapping_diag.get('map_live') is True)
        )
        robot = self.engine.state.get('robots', {}).get(rid, {})
        control_mode = str(robot.get('control_mode') or '').upper()
        control_request = getattr(self, 'control_mode_requests', {}).get(rid, {})
        command_state = self.robot_command_diagnostics.get(rid, {})
        applied_mode = str(command_state.get('active_control_mode') or '').upper()
        estop_clear = command_fresh and command_state.get('estop_active') is False
        control_applied = bool(
            command_fresh and applied_mode == control_mode
            and control_request.get('mode_transition_state') != 'REQUESTED'
            and control_request.get('applied_mode') in (None, applied_mode)
        )
        manual_available = bool(
            mapping_available and control_mode == 'MANUAL' and control_applied and estop_clear
        )
        goal_available = bool(
            nav2_ready and map_ready and control_mode == 'AUTONOMOUS'
            and control_applied and estop_clear and rid not in self.local_map_transitions
            and self.navigation_localization_state(rid, active_map) is not None
        )
        tag_navigation_available = bool(
            goal_available and active_map.get('active_map_id') == 'CANONICAL'
            and active_map.get('map_sync_status') == 'CANONICAL'
        )
        return {
            'mapping_available': mapping_available,
            'mapping_active': mapping_active,
            'nav2_available': nav2_available,
            'nav2_ready': nav2_ready,
            'manual_available': manual_available,
            'goal_available': goal_available,
            'map_ready': map_ready,
            'tag_navigation_available': tag_navigation_available,
        }

    def unified_navigation_blocker(self, robot_id: str) -> str | None:
        """Return a fail-closed reason when the unified Nav2 path is not ready."""
        rid = str(robot_id or '').strip()
        mode = str(self.robot_runtime_modes.get(rid) or self.operation_mode).upper()
        if mode != 'UNIFIED':
            return None
        capabilities = self.robot_capabilities(rid)
        if not capabilities['nav2_ready']:
            return 'NAV2_NOT_READY: Nav2 lifecycle or required action servers are not ready'
        if rid in self.local_map_transitions:
            return 'MAP_TRANSITION: navigation is blocked while the active map is changing'
        command = self.robot_command_diagnostics.get(rid, {})
        command_received = command.get('received_monotonic')
        timeout = float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
        if command_received is None or time.monotonic() - float(command_received) > timeout:
            return 'COMMAND_STATE_STALE: fresh command arbiter mode and E-STOP diagnostics are required'
        robot = self.engine.state.get('robots', {}).get(rid, {})
        request = getattr(self, 'control_mode_requests', {}).get(rid, {})
        if (str(robot.get('control_mode') or '').upper() != 'AUTONOMOUS'
                or str(command.get('active_control_mode') or '').upper() != 'AUTONOMOUS'
                or request.get('mode_transition_state') == 'REQUESTED'):
            return 'CONTROL_MODE_NOT_AUTONOMOUS: apply AUTONOMOUS before requesting a Nav2 path'
        if command.get('estop_active') is not False:
            return 'ESTOP_STATE_UNCONFIRMED: Nav2 is blocked until runtime confirms E-STOP is clear'
        if not capabilities['map_ready']:
            return 'SLAM_MAP_NOT_READY: the active live map is not available'
        if self.navigation_localization_state(rid, self.active_map_state(rid)) is None:
            return 'LOCALIZATION_NOT_READY: a fresh active-map TF pose is required'
        return None

    def navigation_localization_state(self, robot_id: str, active_map: dict[str, Any]) -> dict[str, str] | None:
        rid = str(robot_id)
        timeout = float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
        pose_heartbeat = self.robot_pose_heartbeats.get(rid)
        if pose_heartbeat is None or time.monotonic() - pose_heartbeat > timeout:
            return None
        robot = self.engine.state.get('robots', {}).get(rid)
        pose = robot.get('active_map_pose') if isinstance(robot, dict) else None
        if not isinstance(pose, dict) or pose.get('valid') is not True:
            return None
        signature = {
            'frame_id': str(pose.get('frame_id') or ''),
            'map_id': str(pose.get('map_id') or ''),
            'map_revision': str(pose.get('map_revision') or ''),
            'map_source': str(pose.get('map_source') or ''),
            'pose_source': str(pose.get('pose_source') or ''),
        }
        if (signature['frame_id'] != 'map'
                or signature['map_id'] != str(active_map.get('active_map_id') or '')
                or signature['map_revision'] != str(active_map.get('active_map_revision') or '')
                or signature['pose_source'] != 'TF'):
            return None
        return signature

    async def resolve_navigation_target(self, **kwargs) -> dict[str, Any]:
        from .navigation_targets import resolve_navigation_target
        return await sync_to_async(resolve_navigation_target, thread_sensitive=True)(**kwargs)

    def invalidate_path_previews(self, robot_id: str, reason: str = 'robot control state changed') -> None:
        rid = str(robot_id or '').strip()
        keys = {key for cache in (self.path_preview_requests, self.approved_path_previews,
                                  self.path_preview_results) for key in cache if key[0] == rid}
        for key in keys:
            if key in self.path_preview_results or key in self.approved_path_previews:
                self.path_preview_invalidations[key] = reason
            for cache in (self.path_preview_requests, self.approved_path_previews,
                          self.path_preview_results):
                cache.pop(key, None)

    async def ensure_started(self) -> None:
        if self.channel_layer is None:
            self.channel_layer = get_channel_layer()
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self.run(), name='waretwin-django-runtime')

    async def run(self) -> None:
        acc = 0.0
        last = time.perf_counter()
        rate_n, rate_t = 0, last
        while True:
            await asyncio.sleep(0.01)
            now = time.perf_counter()
            dt = min(0.25, now - last)
            last = now
            if not self.is_external and (self.paused or self.speed == 0):
                continue
            # External modes must keep watchdog/scheduler time alive even if a
            # frontend sends a simulation pause/speed command.  Those controls
            # are meaningful only for LOCAL_SIM.
            acc += dt if self.is_external else dt * self.speed
            n = 0
            try:
                while acc >= TICK_S and n < 40:
                    if self.is_external:
                        # In external modes this is only a web/runtime clock and
                        # cache maintenance tick; it never moves a robot.
                        self.engine.state['sim']['tick'] += 1
                    else:
                        self.engine.step()
                    self.plc.tick(TICK_S)
                    self.engine.state['conveyors'] = self.plc.snapshot()
                    acc -= TICK_S
                    n += 1
                if n:
                    rate_n += n
                    if now - rate_t >= 1:
                        self.tick_rate_actual = rate_n / (now - rate_t)
                        rate_n, rate_t = 0, now
                    await self.after_ticks()
                    self.last_progress = time.monotonic()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.loop_errors += 1
                self.last_error = f'{type(exc).__name__}: {exc}'[:300]
                log.exception('runtime loop error')
                acc = 0.0
                await asyncio.sleep(0.25)

    @profile_async
    async def after_ticks(self) -> None:
        S = self.engine.state
        S['sim']['speed'] = self.speed
        S['sim']['mode'] = 'LIVE' if self.is_external else ('PAUSED' if self.paused else 'LIVE')
        # Database-backed fleet schedules belong to the retired warehouse
        # product and must never command an external robot. In this application
        # external motion is authorized only by the explicit manual and
        # validated Nav2 control paths. Keep the historical scheduler available
        # solely to the isolated local simulation runtime.
        if S['sim']['tick'] % 10 == 0 and not self.is_external:
            try:
                from .schedule_services import process_runtime_schedules
                changed = await sync_to_async(process_runtime_schedules, thread_sensitive=True)(self.engine, self.plc)
                if changed:
                    await self.broadcast({'type': 'SCHEDULE_UPDATED', 'source': 'runtime'})
            except Exception as exc:
                self.loop_errors += 1
                self.last_error = f'scheduler: {type(exc).__name__}: {exc}'[:300]
                log.exception('scheduler loop error')
        events = self.engine.new_events
        self.engine.new_events = []
        tick = S['sim']['tick']
        if events:
            try:
                await sync_to_async(_persist_events, thread_sensitive=True)(self.run_id, events)
            except Exception as exc:
                self.loop_errors += 1
                self.last_error = f'db: {type(exc).__name__}: {exc}'[:300]
        if self.is_external and self.last_telemetry_at is not None:
            timeout = float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
            if time.monotonic() - self.last_telemetry_at > timeout:
                await self._mark_external_offline()
        if self.client_count <= 0:
            self._snapshot_prev()
            self.last_sent_tick = tick
            return
        patch = self.make_patch()
        await self.broadcast({'type': 'PATCH', 'base_tick': self.last_sent_tick, 'tick': tick, 'patch': patch, 'events': events})
        self.last_sent_tick = tick
        if tick % HEATMAP_EVERY == 0:
            for floor, values in self.engine.traffic.items():
                await self.broadcast({'type': 'HEATMAP', 'layer': self.heatmap_layer('CONGESTION', values, floor)})
                await self.broadcast({'type': 'HEATMAP', 'layer': self.heatmap_layer('TRAFFIC', self.engine.traffic_short[floor], floor)})
    async def broadcast(self, payload: dict[str, Any]) -> None:
        from .visualization_outbox import VISUALIZATION_TYPES
        if payload.get('type') in VISUALIZATION_TYPES:
            # Avoid Channels' deep-copy/FIFO backlog for disposable clouds.
            # Each authenticated consumer owns one bounded latest-only lane.
            for client in tuple(self.visualization_clients):
                client.visualization_outbox.offer(payload)
            return
        if self.channel_layer is None:
            self.channel_layer = get_channel_layer()
        if self.channel_layer is not None:
            await self.channel_layer.group_send('twin_clients', {'type': 'twin.message', 'payload': payload})

    async def broadcast_full(self) -> None:
        """Broadcast an authoritative snapshot after an operator mutation.

        This is especially important while the simulation is paused, because there
        may be no next tick to carry a PATCH.
        """
        tick = self.engine.state['sim']['tick']
        self._snapshot_prev()
        self.last_sent_tick = tick
        await self.broadcast(self.full_message())

    async def broadcast_runtime_status(self) -> None:
        await self.broadcast(self.runtime_status_message())

    def runtime_status_message(self) -> dict[str, Any]:
        from .map_sync import map_sync_status
        if self.robot_map_sync:
            self._aggregate_robot_map_sync()
        require_nav2, require_tag_map = self.map_sync_requirements()
        return {
            'type': 'RUNTIME_STATUS', 'runtime_mode': self.runtime_mode,
            'runtime_state': self.operation_mode,
            'bridge_state': self.bridge_status,
            'ros_connected': bool(self.online_robot_ids()),
            'connected_robot_ids': self.online_robot_ids(),
            'nav2_state': self.nav2_state,
            'last_telemetry_at': self.last_telemetry_iso,
            'published_revision': self.published_map_revision,
            'published_version': self.published_map_version,
            'ros_revision': self.ros_map_revision,
            'gazebo_revision': self.gazebo_map_revision,
            'nav2_revision': self.nav2_map_revision,
            'tag_map_revision': self.tag_map_revision,
            'tf_status': self.map_tf_status,
            'robot_map_sync': {
                rid: {key: value for key, value in row.items() if key != 'received_monotonic'}
                for rid, row in self.robot_map_sync.items()
            },
            'local_active_maps': {
                rid: self.active_map_state(rid)
                for rid in sorted(self.connected_robot_ids | set(self.local_map_overrides))
            },
            'robot_capabilities': {
                rid: self.robot_capabilities(rid)
                for rid in sorted(self.connected_robot_ids | set(self.local_map_overrides))
            },
            'robot_mapping_sessions': {
                rid: self.robot_mapping_sessions.get(rid)
                for rid in sorted(self.connected_robot_ids)
            },
            'map_sync_status': map_sync_status(
                published_revision=self.published_map_revision,
                ros_revision=self.ros_map_revision,
                gazebo_revision=self.gazebo_map_revision,
                nav2_revision=self.nav2_map_revision,
                tag_map_revision=self.tag_map_revision,
                tf_status=self.map_tf_status,
                require_nav2=require_nav2,
                require_tag_map=require_tag_map,
                ros_connected=self.ros_bridge_connected,
                error=self.map_sync_error,
                external=self.is_external,
            ),
            'map_sync_error': self.map_sync_error,
            'diagnostics': self.ros_diagnostics,
        }

    async def bridge_connected(self, robot_id: str | None = None) -> None:
        if robot_id:
            rid = str(robot_id)
            self.connected_robot_ids.add(rid)
            self.robot_command_diagnostics.pop(rid, None)
            self.command_ownership.pop(rid, None)
            self.robot_bridge_heartbeats[rid] = time.monotonic()
            self.robot_pose_heartbeats.pop(rid, None)
            self.robot_map_sync.pop(rid, None)
            self.invalidate_path_previews(rid)
            self._aggregate_robot_map_sync()
        self.ros_bridge_connected = bool(self.connected_robot_ids)
        self.bridge_status = 'CONNECTED'
        self.ros_diagnostics['ros'] = True
        self.nav2_state = 'CONNECTED'
        self.map_sync_error = None
        await self.send_published_map_to_bridge()
        await self.broadcast_runtime_status()

    async def bridge_disconnected(self, robot_id: str | None = None) -> None:
        if robot_id:
            rid = str(robot_id)
            self.connected_robot_ids.discard(rid)
            self.robot_command_diagnostics.pop(rid, None)
            self.command_ownership.pop(rid, None)
            self.robot_bridge_heartbeats.pop(rid, None)
            self.robot_pose_heartbeats.pop(rid, None)
            self.robot_map_sync.pop(rid, None)
            self._aggregate_robot_map_sync()
        else:
            self.connected_robot_ids.clear()
            self.robot_command_diagnostics.clear()
            self.command_ownership.clear()
            self.robot_bridge_heartbeats.clear()
            self.robot_pose_heartbeats.clear()
            self.robot_map_sync.clear()
        self.ros_bridge_connected = bool(self.connected_robot_ids) if robot_id else False
        if self.ros_bridge_connected:
            # One namespaced bridge went away; keep the other robots and the
            # global ROS connection healthy.
            await self.broadcast_runtime_status()
            return
        self.bridge_status = 'DISCONNECTED'
        self.nav2_state = 'OFFLINE'
        self.operation_mode = 'ERROR' if self.is_external else self.operation_mode
        self.ros_diagnostics = {
            **self.ros_diagnostics,
            'ros': False, 'gazebo': False, 'controller_manager': False,
            'slam': False, 'nav2': False, 'tf': False, 'lidar': False,
            'nodes': [], 'topics': [], 'controllers': [],
        }
        self.ros_map_revision = None
        self.gazebo_map_revision = None
        self.nav2_map_revision = None
        self.tag_map_revision = None
        self.map_sync_error = None
        if self.is_external:
            await self._mark_external_offline()
        await self.broadcast_runtime_status()

    async def send_published_map_to_bridge(self) -> bool:
        """Send only the immutable published revision to the connected bridge."""
        from .map_sync import published_map_payload
        from .ros_bridge_consumer import registry
        payload = await sync_to_async(published_map_payload, thread_sensitive=True)()
        self.published_map_revision = payload.get('map_revision')
        self.published_map_version = int(payload.get('published_version') or 0)
        if payload.get('artifact_dir'):
            payload['type'] = 'MAP_PUBLISHED'
            return await registry.send(payload)
        return False

    async def map_published(self, payload: dict[str, Any]) -> None:
        """Broadcast a publish event and push the revision to ROS."""
        self.published_map_revision = payload.get('map_revision', payload.get('revision'))
        self.published_map_version = int(payload.get('published_version') or 0)
        self.map_sync_error = None
        await self.broadcast({
            'type': 'map.published',
            'warehouse_id': payload.get('warehouse_id'),
            'revision': self.published_map_revision,
            'published_version': self.published_map_version,
            'map_revision': self.published_map_revision,
            'artifact_manifest': payload.get('artifact_manifest'),
        })
        from .ros_bridge_consumer import registry
        ros_payload = dict(payload)
        ros_payload['type'] = 'MAP_PUBLISHED'
        sent = await registry.send(ros_payload)
        if not sent and self.is_external:
            self.map_sync_error = 'ROS bridge is not connected'
        await self.broadcast_runtime_status()

    async def handle_map_revision_status(self, data: dict[str, Any]) -> None:
        rid = str(data.get('robot_id') or '').strip()
        if rid and rid not in self.connected_robot_ids:
            return
        if rid:

            def revision(*keys):
                raw_value = next((data.get(key) for key in keys if data.get(key) is not None), None)
                try:
                    return int(raw_value) if raw_value is not None else None
                except (TypeError, ValueError):
                    return None

            previous = self.robot_map_sync.get(rid, {})
            next_revision = revision('ros_revision', 'map_revision', 'revision')
            self.robot_map_sync[rid] = {
                'ros_revision': next_revision,
                'gazebo_revision': revision('gazebo_revision', 'map_revision', 'revision'),
                'nav2_revision': revision('nav2_revision'),
                'tag_map_revision': revision('tag_map_revision'),
                'tf_status': bool(data.get('tf_status', False)),
                'nav2_required': bool(data.get('nav2_required', self.operation_mode == 'NAVIGATION')),
                'tag_map_required': bool(data.get('tag_map_required', self.operation_mode == 'NAVIGATION')),
                'error': str(data.get('error') or '') or None,
                'received_monotonic': time.monotonic(),
                'status': str(data.get('status') or 'OUT_OF_SYNC'),
            }
            if previous.get('ros_revision') != next_revision and not self.local_map_overrides.get(rid):
                self.invalidate_path_previews(rid, 'canonical map revision changed')
            self._aggregate_robot_map_sync()
        else:
            # Compatibility for older single-robot bridge clients.
            raw = data.get('ros_revision', data.get('map_revision', data.get('revision')))
            try: self.ros_map_revision = int(raw) if raw is not None else None
            except (TypeError, ValueError): self.ros_map_revision = None
            raw_gazebo = data.get('gazebo_revision', raw)
            try: self.gazebo_map_revision = int(raw_gazebo) if raw_gazebo is not None else None
            except (TypeError, ValueError): self.gazebo_map_revision = None
            try: self.nav2_map_revision = int(data.get('nav2_revision')) if data.get('nav2_revision') is not None else None
            except (TypeError, ValueError): self.nav2_map_revision = None
            try: self.tag_map_revision = int(data.get('tag_map_revision')) if data.get('tag_map_revision') is not None else None
            except (TypeError, ValueError): self.tag_map_revision = None
            self.map_tf_status = bool(data.get('tf_status', False))
            self.map_sync_error = str(data.get('error') or '') or None
        await self.broadcast_runtime_status()
        # The bridge owns revision reload requests and its supervisor restarts
        # consumers. Re-sending on every heartbeat creates a retry storm while
        # TF or Nav2 is still initializing.

    def _aggregate_robot_map_sync(self) -> None:
        """Require every connected robot bridge to report the same map state."""
        robot_ids = sorted(self.connected_robot_ids)
        rows = {rid: self.robot_map_sync[rid] for rid in robot_ids if rid in self.robot_map_sync}
        if not rows:
            self.ros_map_revision = self.gazebo_map_revision = None
            self.nav2_map_revision = self.tag_map_revision = None
            self.map_tf_status = False
            self.map_sync_error = 'waiting for robot map revision status' if robot_ids else None
            return
        stale_after = float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
        fresh_rows = {
            rid: row for rid, row in rows.items()
            if time.monotonic() - row['received_monotonic'] <= stale_after
        }

        def common_revision(key):
            values = [fresh_rows[rid].get(key) for rid in robot_ids if rid in fresh_rows]
            if len(values) != len(robot_ids) or not values or any(value != values[0] for value in values):
                return None
            return values[0]

        self.ros_map_revision = common_revision('ros_revision')
        self.gazebo_map_revision = common_revision('gazebo_revision')
        self.nav2_map_revision = common_revision('nav2_revision')
        self.tag_map_revision = common_revision('tag_map_revision')
        self.map_tf_status = len(fresh_rows) == len(robot_ids) and all(
            row.get('tf_status', False) for row in fresh_rows.values())
        errors = [f'{rid}: {row["error"]}' for rid, row in rows.items() if row.get('error')]
        missing = sorted(set(robot_ids) - set(fresh_rows))
        if missing:
            errors.append('stale/missing map status from ' + ', '.join(missing))
        keys = ('ros_revision', 'gazebo_revision', 'nav2_revision', 'tag_map_revision')
        mismatched = [key for key in keys if len({row.get(key) for row in fresh_rows.values()}) > 1]
        if mismatched:
            errors.append('robot bridges disagree on ' + ', '.join(mismatched))
        self.map_sync_error = '; '.join(errors) or None

    async def _mark_external_offline(self) -> None:
        changed = False
        for robot in self.engine.state.get('robots', {}).values():
            if robot.get('status') != 'OFFLINE':
                robot['status'] = 'OFFLINE'
                robot['fsm'] = 'OFFLINE'
                robot['navigation_state'] = 'OFFLINE'
                robot['vx'] = robot['vy'] = robot['wz'] = 0.0
                changed = True
        if changed and self.client_count:
            await self.broadcast({'type': 'PATCH', 'base_tick': self.last_sent_tick,
                                  'tick': self.engine.state['sim']['tick'],
                                  'patch': {'robots': self._robot_patch()}, 'events': []})

    async def update_external_robot_state(self, data: dict[str, Any]) -> None:
        rid = str(data.get('robot_id') or '').strip()
        if not rid or rid not in self.connected_robot_ids:
            return
        frame_id = str(data.get('frame_id') or '')
        if frame_id != 'map':
            self.map_tf_status = False
            self.map_sync_error = f'robot {rid} pose frame must be map, received {frame_id or "empty"}'
            return
        active = self.active_map_state(rid)
        reported_map_id = str(data.get('active_map_id') or 'CANONICAL')
        reported_active_revision = str(data.get('active_map_revision') or '')
        reported_map_source = str(data.get('map_source') or '')
        if not reported_map_source:
            if reported_map_id.startswith('SLAM-'):
                reported_map_source = 'SLAM_TOOLBOX'
            elif reported_map_id == 'CANONICAL':
                reported_map_source = 'CANONICAL'
            elif active.get('local_active_map_id') == reported_map_id:
                reported_map_source = 'LOCAL_MAP'
        # ROBOT_STATE is emitted only after the bridge resolves map -> base via tf2.
        # The fallback labels older bridge payloads while retaining frame/map identity.
        pose_source = str(data.get('pose_source') or 'TF')
        robot_mode = str(self.robot_runtime_modes.get(rid) or self.operation_mode).upper()
        if robot_mode in MAPPING_CAPABLE_MODES:
            # Mapping poses are display-only in the live SLAM frame. They are
            # accepted only from the authenticated bridge's SLAM session and
            # never treated as canonical-map or navigation authorization.
            mapping_session = str(self.robot_mapping_sessions.get(rid) or '')
            if (not mapping_session
                    or str(data.get('mapping_session_id') or '') != mapping_session
                    or reported_map_id != f'SLAM-{mapping_session}'
                    or not reported_active_revision):
                return
        elif active.get('local_active_map_id'):
            if (reported_map_id != active.get('active_map_id')
                    or reported_active_revision != active.get('active_map_revision')):
                return
        else:
            try:
                pose_revision = int(data.get('map_revision'))
            except (TypeError, ValueError):
                self.map_sync_error = f'robot {rid} pose is missing its map revision'
                return
            if (self.published_map_revision is None
                    or pose_revision != self.published_map_revision
                    or reported_map_id != 'CANONICAL'
                    or reported_active_revision != str(pose_revision)):
                self.map_sync_error = (f'robot {rid} pose revision {pose_revision} does not match '
                                       f'published revision {self.published_map_revision}')
                return
        robots = self.engine.state.setdefault('robots', {})
        robot = robots.get(rid)
        if robot is None:
            # Preserve the existing schema shape for a robot discovered later.
            robot = next(iter(robots.values()), {}).copy()
            robot.update({'id': rid, 'position': [0.0, 0.0, 0.0], 'path': [], 'path_index': 0,
                          'stats': {'distance_m': 0.0, 'tasks_completed': 0, 'energy_wh': 0.0,
                                    'busy_ticks': 0, 'wait_ticks': 0}, 'load': {'current': 0, 'capacity': 4}})
            robots[rid] = robot
        pose = ros_pose_to_waretwin(float(data.get('x', 0.0)), float(data.get('y', 0.0)),
                                    float(data.get('z', 0.0)), float(data.get('yaw', 0.0)))
        twist = ros_twist_to_waretwin(float(data.get('vx', 0.0)), float(data.get('vy', 0.0)),
                                      float(data.get('wz', 0.0)))
        now_iso = str(data.get('timestamp') or datetime.now(timezone.utc).isoformat())
        active_pose = {'x': float(data['x']), 'y': float(data['y']), 'yaw': float(data['yaw']),
                       'frame_id': frame_id, 'map_id': reported_map_id,
                       'map_revision': reported_active_revision, 'map_source': reported_map_source,
                       'pose_source': pose_source, 'mapping_session_id': data.get('mapping_session_id'),
                       'timestamp': now_iso, 'valid': True}
        canonical_pose = validated_canonical_pose(data.get('canonical_pose'),
                                                  self.published_map_revision, self.runtime_mode)
        if robot_mode not in MAPPING_CAPABLE_MODES and reported_map_id == 'CANONICAL' and pose_source == 'TF':
            canonical_pose = active_pose
        nav = str(data.get('navigation_state') or 'IDLE').upper()
        control_mode = str(data.get('control_mode') or robot.get('control_mode') or 'AUTONOMOUS').upper()
        if control_mode not in ('MANUAL', 'AUTONOMOUS'):
            control_mode = 'AUTONOMOUS'
        reported_battery = data.get('battery')
        if (isinstance(reported_battery, (int, float)) and not isinstance(reported_battery, bool)
                and math.isfinite(float(reported_battery)) and 0 <= float(reported_battery) <= 100):
            robot['battery'] = float(reported_battery)
            robot['battery_reported'] = True
        # ``robot`` is the persisted state dictionary.  ``dict.update`` takes
        # one mapping, so merge the ROS pose, twist and metadata explicitly;
        # the previous three-argument call raised on every ROBOT_STATE frame
        # and silently prevented external telemetry from reaching the Web UI.
        robot.update(pose)
        robot.update(twist)
        robot.update({'navigation_state': nav, 'last_telemetry_at': now_iso,
                      'control_mode': control_mode, 'status': 'ACTIVE',
                      'fsm': self._fsm_from_nav(nav),
                      'active_map_pose': active_pose,
                      'slam_pose': active_pose if reported_map_source == 'SLAM_TOOLBOX' else None,
                      'canonical_pose': canonical_pose,
                      'pose_frame_id': frame_id,
                      'pose_map_id': reported_map_id,
                      'pose_map_revision': reported_active_revision or str(data.get('map_revision') or ''),
                      'pose_map_source': reported_map_source,
                      'pose_source': pose_source,
                      'pose_mapping_session_id': str(data.get('mapping_session_id') or '') or None,
                      'battery_reported': robot.get('battery_reported') is True})
        self.last_telemetry_at = time.monotonic()
        self.last_ros_heartbeat = self.last_telemetry_at
        self.robot_bridge_heartbeats[rid] = self.last_telemetry_at
        self.robot_pose_heartbeats[rid] = self.last_telemetry_at
        self.bridge_status = 'CONNECTED'
        self.ros_diagnostics['ros'] = True
        self.last_telemetry_iso = now_iso
        # The wall-clock runtime tick publishes coalesced state deltas. Never
        # await browser group delivery on the ROS ingress path: a slow browser
        # must not hold subsequent heartbeats, view ACKs or control status.

    @staticmethod
    def _fsm_from_nav(nav: str) -> str:
        return {'ACTIVE': 'NAVIGATING', 'NAVIGATING': 'NAVIGATING', 'SUCCEEDED': 'COMPLETED',
                'FAILED': 'ERROR', 'CANCELED': 'IDLE', 'IDLE': 'IDLE'}.get(nav, 'NAVIGATING')

    async def handle_nav_status(self, data: dict[str, Any]) -> None:
        from .schedule_services import apply_external_nav_status
        changed = await sync_to_async(apply_external_nav_status, thread_sensitive=True)(data)
        self.nav2_state = str(data.get('status') or self.nav2_state)
        if changed:
            await self.broadcast({'type': 'SCHEDULE_UPDATED', 'source': 'ros-nav2'})

    async def handle_ros_message(self, data: dict[str, Any]) -> None:
        kind = str(data.get('type') or '').upper()
        if kind == 'ROBOT_STATE':
            await self.update_external_robot_state(data)
        elif kind == 'NAV_STATUS':
            await self.handle_nav_status(data)
            # Keep the control console independent from scheduler persistence;
            # the browser needs the measured Nav2 state even when no schedule
            # row is associated with a manually selected map goal.
            await self.broadcast(data)
        elif kind == 'HEARTBEAT':
            robot_id = str(data.get('robot_id') or '').strip()
            if not robot_id:
                return
            if robot_id not in self.connected_robot_ids:
                # The authenticated bridge socket can outlive an in-memory
                # runtime reset/reload. Its consumer identity is established
                # by RosBridgeConsumer.connect(), which validates the bridge
                # token and overwrites any reported robot_id before dispatch.
                # Let only that still-registered socket restore its own ID;
                # arbitrary heartbeat packets must never create fleet entries.
                from .ros_bridge_consumer import registry
                bridge = registry.consumers.get(robot_id)
                if bridge is None or str(getattr(bridge, 'robot_id', robot_id)) != robot_id:
                    return
                self.connected_robot_ids.add(robot_id)
            self.last_ros_heartbeat = time.monotonic()
            self.robot_bridge_heartbeats[robot_id] = self.last_ros_heartbeat
            self.ros_bridge_connected = bool(self.connected_robot_ids)
            self.bridge_status = str(data.get('bridge_state') or 'CONNECTED').upper()
            self.nav2_state = str(data.get('nav2_state') or 'CONNECTED')
            previous_mode = self.robot_runtime_modes.get(robot_id)
            incoming_mode = str(data.get('runtime_state') or self.operation_mode).upper()
            mapping_session = str(data.get('mapping_session_id') or '')
            previous_mapping_session = self.robot_mapping_sessions.get(robot_id)
            cached_snapshot = self.robot_map_snapshots.get(robot_id, {})
            cached_map = cached_snapshot.get('map') if isinstance(cached_snapshot, dict) else {}
            cached_slam_snapshot = self.robot_slam_map_snapshots.get(robot_id, {})
            cached_slam_map = cached_slam_snapshot.get('map') if isinstance(cached_slam_snapshot, dict) else {}
            current_session_snapshot = (
                incoming_mode in MAPPING_CAPABLE_MODES
                and isinstance(cached_slam_map, dict)
                and cached_slam_map.get('map_source') == 'SLAM_TOOLBOX'
                and mapping_session
                and cached_slam_map.get('mapping_session_id') == mapping_session
            )
            if (previous_mode != incoming_mode
                    or (incoming_mode in MAPPING_CAPABLE_MODES
                        and mapping_session and mapping_session != previous_mapping_session)) \
                    and not current_session_snapshot:
                # Never replay a map snapshot from the previous runtime/map
                # source into a new mapping or navigation session.
                self.robot_map_snapshots.pop(robot_id, None)
                self.robot_runtime_map_snapshots.pop(robot_id, None)
                self.robot_slam_map_snapshots.pop(robot_id, None)
                self.robot_map_geometry.pop(robot_id, None)
            self.robot_runtime_modes[robot_id] = incoming_mode
            if incoming_mode in MAPPING_CAPABLE_MODES and mapping_session:
                self.robot_mapping_sessions[robot_id] = mapping_session
            elif incoming_mode not in MAPPING_CAPABLE_MODES:
                self.robot_mapping_sessions.pop(robot_id, None)
            self.operation_mode = incoming_mode
            if data.get('mapping_state'):
                self.robot_mapping_state[robot_id] = str(data['mapping_state']).upper()
            try:
                self.robot_mapping_elapsed_s[robot_id] = max(0.0, float(data.get('mapping_elapsed_s', 0.0)))
            except (TypeError, ValueError):
                self.robot_mapping_elapsed_s[robot_id] = 0.0
            if self.operation_mode not in ('IDLE', 'SIMULATION', 'MAPPING', 'NAVIGATION', 'UNIFIED', 'ERROR'):
                self.operation_mode = 'SIMULATION' if self.is_external else self.operation_mode
            self.ros_diagnostics['ros'] = True
            self.ros_diagnostics['last_update_at'] = data.get('timestamp')
            await self.broadcast_runtime_status()
        elif kind in ('ROS_DIAGNOSTICS', 'DIAGNOSTICS'):
            await self.handle_ros_diagnostics(data)
        elif kind == 'SYSTEM_DIAGNOSTICS':
            # Some bridge versions emit the richer packet without a separate
            # ROS_DIAGNOSTICS frame.  Ingest it as measured runtime health as
            # well as forwarding the robot-scoped console payload.
            await self.handle_ros_diagnostics(data)
            await self.broadcast(data)
        elif kind == 'PATH_PREVIEW_RESULT':
            robot_id = str(data.get('robot_id') or '')
            request_id = str(data.get('request_id') or '')
            key = (robot_id, request_id)
            request = self.path_preview_requests.pop(key, None)
            if request is None:
                return
            now = time.monotonic()
            if now - float(request.get('created_monotonic', now)) > 120.0:
                self.expired_path_previews[key] = now
                return

            status = str(data.get('status') or 'INVALID').upper()
            path = data.get('path') if isinstance(data.get('path'), list) else []
            target = data.get('goal') if isinstance(data.get('goal'), dict) else {}
            try:
                path_is_valid = bool(path) and all(
                    isinstance(point, (list, tuple)) and len(point) >= 2
                    and all(math.isfinite(float(value)) for value in point[:2])
                    for point in path
                )
            except (TypeError, ValueError):
                path_is_valid = False
            try:
                target_matches = all(
                    math.isfinite(float(target.get(axis, float('nan'))))
                    and abs(float(target.get(axis)) - float(request['goal'][axis])) <= 1e-4
                    for axis in ('x', 'y', 'yaw')
                )
            except (KeyError, TypeError, ValueError):
                target_matches = False
            map_matches = (
                str(data.get('active_map_id') or '') == request['active_map_id']
                and str(data.get('active_map_revision') or '') == request['active_map_revision']
            )
            active_now = self.active_map_state(robot_id)
            still_current = (
                active_now.get('active_map_id') == request['active_map_id']
                and active_now.get('active_map_revision') == request['active_map_revision']
                and self.navigation_localization_state(robot_id, active_now) == request['localization_state']
            )
            source_still_current = True
            if request.get('source_type') == 'TAG':
                try:
                    current_target = await self.resolve_navigation_target(
                        robot_id=robot_id, source_type='TAG', active_map=active_now,
                        tag_id=request.get('tag_id'),
                    )
                    current_metadata = current_target.get('metadata') or {}
                    source_still_current = (
                        str(current_target.get('source_id')) == str(request.get('source_id'))
                        and current_metadata.get('tag_revision') == request.get('tag_revision')
                        and current_metadata.get('registry_revision') == request.get('registry_revision')
                        and all(abs(float(current_target[axis]) - float(request['goal'][axis])) <= 1e-4
                                for axis in ('x', 'y', 'yaw'))
                    )
                except Exception as exc:
                    source_still_current = False
                    data['reason'] = f'Tag target could not be revalidated ({type(exc).__name__})'
            if status == 'VALID' and not (path_is_valid and target_matches and map_matches and still_current and source_still_current):
                status = 'INVALID'
                data.setdefault('reason', 'planner result did not match the requested target, map, or localization state')
            data.update({
                'status': status,
                'source_type': request['source_type'],
                'source_id': request.get('source_id'),
                'tag_id': request.get('tag_id'),
                'tag_revision': request.get('tag_revision'),
                'registry_revision': request.get('registry_revision'),
                'active_map_id': request['active_map_id'],
                'active_map_revision': request['active_map_revision'],
                'canonical_map_revision': request['canonical_map_revision'],
                'goal': {**request['goal'], 'frame_id': 'map'},
            })
            record = {
                **request,
                'status': status,
                'path_found': status == 'VALID' and path_is_valid,
                'created_monotonic': now,
                'goal': request['goal'],
            }
            self.path_preview_results[key] = record
            if record['path_found']:
                self.approved_path_previews[key] = record.copy()
            else:
                self.approved_path_previews.pop(key, None)
            for cache in (self.path_preview_results, self.approved_path_previews):
                for cached_key, value in list(cache.items()):
                    if now - float(value.get('created_monotonic', now)) > 120.0:
                        cache.pop(cached_key, None)
                        self.expired_path_previews[cached_key] = now
            for expired_key, marked_at in list(self.expired_path_previews.items()):
                if now - marked_at > 120.0:
                    self.expired_path_previews.pop(expired_key, None)
            if len(self.path_preview_results) > 256:
                oldest = sorted(self.path_preview_results, key=lambda item: self.path_preview_results[item]['created_monotonic'])[:-256]
                for old_key in oldest:
                    self.path_preview_results.pop(old_key, None)
                    self.approved_path_previews.pop(old_key, None)
            await self.broadcast(data)
        elif kind == 'MAP_SNAPSHOT':
            map_data = data.get('map') if isinstance(data.get('map'), dict) else {}
            rid = str(map_data.get('robot_id') or data.get('robot_id') or '')
            try:
                resolution = float(map_data.get('resolution'))
                width = int(map_data.get('width'))
                height = int(map_data.get('height'))
                origin = map_data.get('origin') or {}
                origin_x, origin_y, origin_yaw = (float(origin[key]) for key in ('x', 'y', 'yaw'))
                if resolution > 0 and width > 0 and height > 0 and all(
                        math.isfinite(value) for value in (resolution, origin_x, origin_y, origin_yaw)):
                    self.robot_map_geometry[rid] = {
                        'width': width, 'height': height, 'resolution': resolution,
                        'origin': {'x': origin_x, 'y': origin_y, 'yaw': origin_yaw},
                        'active_map_id': map_data.get('active_map_id'),
                        'active_map_revision': map_data.get('active_map_revision'),
                    }
                    cells = map_data.get('data')
                    compressed_cells = map_data.get('data_zlib_base64')
                    has_cells = (isinstance(cells, list) and len(cells) == width * height)
                    has_compressed_cells = (
                        map_data.get('data_encoding') == 'zlib-base64-offset1'
                        and isinstance(compressed_cells, str)
                        and 0 < len(compressed_cells) <= 5_600_000
                    )
                    if (rid and (has_cells or has_compressed_cells)
                            and width * height <= 4_000_000):
                        # Map snapshots are event-driven at the bridge. Retain
                        # the latest active map for backend map operations, and
                        # retain SLAM independently so reconnect/replay never
                        # conflates accumulated mapping data with runtime maps.
                        retained = {
                            **data,
                            'map': {**map_data, 'robot_id': rid},
                        }
                        self.robot_map_snapshots[rid] = retained
                        if map_data.get('map_source') == 'SLAM_TOOLBOX':
                            self.robot_slam_map_snapshots[rid] = retained
                        else:
                            self.robot_runtime_map_snapshots[rid] = retained
                        if len(self.robot_map_snapshots) > 64:
                            oldest_robot = next(iter(self.robot_map_snapshots))
                            if oldest_robot != rid:
                                self.robot_map_snapshots.pop(oldest_robot, None)
                        if len(self.robot_slam_map_snapshots) > 64:
                            oldest_robot = next(iter(self.robot_slam_map_snapshots))
                            if oldest_robot != rid:
                                self.robot_slam_map_snapshots.pop(oldest_robot, None)
                        for cache in (self.robot_runtime_map_snapshots, self.robot_slam_map_snapshots):
                            if len(cache) > 64:
                                oldest_robot = next(iter(cache))
                                if oldest_robot != rid:
                                    cache.pop(oldest_robot, None)
            except (TypeError, ValueError, KeyError):
                pass
            await self.broadcast(data)
        elif kind in ('LIDAR_SCAN', 'LIDAR_MAP_2D', 'LIDAR_MAP_3D',
                      'ROBOT_DETAIL_VIEW_STATUS',
                      'NAV_GLOBAL_PATH', 'NAV_LOCAL_PATH', 'NAV_GOAL', 'CONTROLLER_STATE'):
            if isinstance(data.get('view_timing'), dict):
                data = {**data, 'view_timing': {**data['view_timing'],
                    'django_frame_received_ms': time.time() * 1000}}
            await self.broadcast(data)
        elif kind == 'LOCAL_CONTROL_RESULT':
            from .ros_bridge_consumer import registry
            registry.resolve_request(data)
        elif kind == 'LOCAL_MAP_STATUS':
            robot_id = str(data.get('robot_id') or '')
            if robot_id:
                map_id = str(data.get('map_id') or '')
                if data.get('loaded') and map_id:
                    self.local_map_overrides[robot_id] = map_id
                    revision = str(data.get('active_map_revision') or data.get('map_revision') or '')
                    if revision:
                        self.local_map_revisions[robot_id] = revision
                    self.local_map_status[robot_id] = {
                        'loaded': True, 'map_id': map_id,
                        'active_map_revision': revision or None,
                        'canonical_map_revision': data.get('canonical_map_revision'),
                        'timestamp': data.get('timestamp'),
                    }
                else:
                    self.local_map_overrides.pop(robot_id, None)
                    self.local_map_revisions.pop(robot_id, None)
                    self.local_map_status[robot_id] = {
                        'loaded': False,
                        'canonical_map_revision': data.get('canonical_map_revision'),
                        'timestamp': data.get('timestamp'),
                    }
                self.invalidate_path_previews(robot_id, 'active map changed')
            await self.broadcast(data)
            await self.broadcast_runtime_status()
        elif kind == 'COMMAND_DIAGNOSTICS':
            robot_id = str(data.get('robot_id') or '')
            if robot_id:
                diagnostics = {
                    key: data.get(key) for key in (
                        'active_command_source', 'active_control_mode', 'last_command_age',
                        'manual_source_active', 'nav_source_active', 'tag_source_active',
                        'estop_active',
                    )
                }
                diagnostics['received_monotonic'] = time.monotonic()
                self.robot_command_diagnostics[robot_id] = diagnostics
                self.command_ownership[robot_id] = {
                    key: value for key, value in diagnostics.items()
                    if key != 'received_monotonic'
                }
            await self.broadcast(data)
        elif kind == 'LIDAR_STREAM_DIAGNOSTICS':
            robot_id = str(data.get('robot_id') or '')
            if robot_id:
                self.robot_lidar_streams[robot_id] = {
                    key: data.get(key) for key in (
                        'source_timestamp', 'send_timestamp', 'frame_id', 'point_count',
                        'source_fps', 'web_output_fps', 'dropped_frames', 'view',
                    )
                }
            await self.broadcast(data)
        elif kind == 'VDA5050_RUNTIME_STATUS':
            await self.broadcast(data)
        elif kind == 'BRIDGE_STATUS':
            robot_id = str(data.get('robot_id') or '').strip()
            if not robot_id or robot_id not in self.connected_robot_ids:
                return
            self.bridge_status = str(data.get('state') or 'ERROR').upper()
            runtime_state = str(data.get('runtime_state') or '').upper()
            if runtime_state in ('IDLE', 'SIMULATION', 'MAPPING', 'NAVIGATION', 'UNIFIED', 'ERROR'):
                self.operation_mode = runtime_state
            await self.broadcast_runtime_status()
        elif kind in ('MAP_REVISION_STATUS', 'MAP_REVISION_ACK'):
            await self.handle_map_revision_status(data)
        elif kind == 'ROBOT_CONTROL_STATUS':
            robot_id = str(data.get('robot_id') or '')
            pending = getattr(self, 'control_mode_requests', {}).get(robot_id)
            if pending and data.get('request_id') != pending['request_id']:
                return
            if pending:
                pending.update({key: data.get(key) for key in ('applied_mode', 'mode_transition_state')})
            if data.get('mode_transition_state') == 'APPLIED' and data.get('accepted'):
                robot = self.engine.state.get('robots', {}).get(robot_id)
                if robot is not None:
                    robot['control_mode'] = data.get('applied_mode')
            await self.broadcast(data)
        elif kind in ('TAG_NAV_STATUS', 'TAG_DETECTION', 'LOCALIZATION_STATUS', 'TAG_NAV_ROUTE', 'TAG_NAV_EVENT'):
            await self.handle_tag_navigation_message(kind, data)

    async def handle_ros_diagnostics(self, data: dict[str, Any]) -> None:
        """Accept measured ROS graph/sensor/controller state from the bridge."""
        values = data.get('diagnostics') if isinstance(data.get('diagnostics'), dict) else data
        for key in ('ros', 'gazebo', 'controller_manager', 'slam', 'nav2', 'nav2_ready', 'tf', 'lidar'):
            if key in values:
                self.ros_diagnostics[key] = bool(values[key])
        for key in ('nodes', 'topics', 'controllers'):
            if isinstance(values.get(key), list):
                self.ros_diagnostics[key] = values[key][:200]
        if isinstance(values.get('metrics'), dict):
            self.ros_diagnostics['metrics'] = dict(values['metrics'])
        if isinstance(values.get('mapping'), dict):
            self.ros_diagnostics['mapping'] = dict(values['mapping'])
        if isinstance(values.get('errors'), list):
            self.ros_diagnostics['errors'] = values['errors'][:100]
        if values.get('simulation_time') is not None:
            self.ros_diagnostics['simulation_time'] = values.get('simulation_time')
        self.ros_diagnostics['last_update_at'] = data.get('timestamp') or datetime.now(timezone.utc).isoformat()
        self.ros_diagnostics_received_monotonic = time.monotonic()
        if self.operation_mode == 'ERROR' and self.ros_diagnostics.get('ros'):
            self.operation_mode = 'SIMULATION' if self.is_external else self.operation_mode
        await self.broadcast_runtime_status()

    def health_snapshot(self) -> dict[str, Any]:
        online_robot_ids = self.online_robot_ids() if self.is_external else []
        ros_connected = bool(online_robot_ids)
        diagnostics = dict(self.ros_diagnostics)
        diagnostics['ros'] = bool(diagnostics.get('ros') and ros_connected)
        gazebo = bool(diagnostics.get('gazebo') and ros_connected)
        # The in-process channel layer is initialized lazily by the first HTTP
        # or WebSocket request.  Health must describe whether the configured
        # WebSocket service is usable, not whether a client happened to connect
        # before this request.
        websocket_ready = bool(getattr(settings, 'CHANNEL_LAYERS', {}))
        return {
            # A socket that stopped sending heartbeats is not healthy even if
            # Channels has not delivered its disconnect callback yet.
            'ros_bridge': ros_connected,
            'online_robot_ids': online_robot_ids,
            'websocket': websocket_ready,
            'ros': diagnostics['ros'],
            'gazebo': gazebo,
            'runtime_state': self.operation_mode,
            'bridge_state': self.bridge_status,
            'ros_connected': ros_connected,
            'diagnostics': diagnostics,
        }

    def map_sync_requirements(self) -> tuple[bool, bool]:
        """Use each bridge's declared runtime contract, with safe nav defaults."""
        rows = list(self.robot_map_sync.values())
        if rows:
            return (any(row.get('nav2_required', False) for row in rows),
                    any(row.get('tag_map_required', False) for row in rows))
        navigation = self.operation_mode == 'NAVIGATION'
        return navigation, navigation

    async def handle_tag_navigation_message(self, kind: str, data: dict[str, Any]) -> None:
        from .tag_navigation import apply_ros_tag_status, apply_ros_tag_event, log_ros_tag_detection, apply_ros_localization, mission_snapshot
        if kind == 'TAG_NAV_STATUS':
            mission = await sync_to_async(apply_ros_tag_status, thread_sensitive=True)(data)
            if mission is not None:
                await self.broadcast({'type': kind, **data, 'mission': mission_snapshot(mission)})
        elif kind == 'TAG_NAV_EVENT':
            mission = await sync_to_async(apply_ros_tag_event, thread_sensitive=True)(data)
            await self.broadcast({'type': kind, **data, 'mission_id': mission.id if mission else data.get('mission_id')})
        elif kind == 'TAG_DETECTION':
            await sync_to_async(log_ros_tag_detection, thread_sensitive=True)(data)
            await self.broadcast({'type': kind, **data})
        elif kind == 'LOCALIZATION_STATUS':
            mission = await sync_to_async(apply_ros_localization, thread_sensitive=True)(data)
            await self.broadcast({'type': kind, **data, 'mission_id': mission.id if mission else data.get('mission_id')})
        else:
            await self.broadcast({'type': kind, **data})

    async def tag_command(self, action: str, mission, user=None) -> dict[str, Any]:
        from .tag_navigation import mission_snapshot
        payload = {'mission_id': mission.id, 'target_tag_id': mission.target_tag.tag_id}
        result = await self.gateway().send_command(mission.robot_id, action, payload)
        if result.get('ok') and action == 'GO_TO_TAG':
            mission.status = 'NAVIGATING'; mission.started_at = mission.started_at or datetime.now(timezone.utc)
            await sync_to_async(mission.save, thread_sensitive=True)()
        await self.broadcast({'type': 'TAG_NAV_STATUS', **mission_snapshot(mission)})
        return result

    def full_message(self) -> dict[str, Any]:
        S = self.engine.state
        S['sim']['speed'] = self.speed
        S['sim']['mode'] = 'LIVE' if self.is_external else ('PAUSED' if self.paused else 'LIVE')
        state = S
        if self.is_external:
            state = {**S, 'robots': {
                rid: {**robot,
                      'battery': robot.get('battery') if robot.get('battery_reported') is True else None,
                      'battery_reported': robot.get('battery_reported') is True}
                for rid, robot in S.get('robots', {}).items()
            }}
        return {'type': 'FULL', 'state': state}

    def _snapshot_prev(self) -> None:
        S = self.engine.state
        for sec in self.SECTIONS:
            self._prev[sec] = {k: json.dumps(v, separators=(',', ':'), sort_keys=True) for k, v in S[sec].items()}
        self._prev_subsys = json.dumps(S['subsystems'], sort_keys=True)
        self._prev['_robots'] = {}

    def _robot_patch(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        prev_sent: dict[str, dict[str, Any]] = self._prev.setdefault('_robots', {})
        tick = self.engine.state['sim']['tick']
        for rid, r in self.engine.state['robots'].items():
            cur = {k: v for k, v in r.items() if k not in ('path', 'position', 'heading', 'velocity', 'battery', 'stats')}
            cur['position'] = [round(r['position'][0], 3), 0, round(r['position'][2], 3)]
            cur['heading'] = round(r['heading'], 3)
            cur['velocity'] = round(r['velocity'], 2)
            cur['vx'] = round(float(r.get('vx', 0.0)), 3)
            cur['vy'] = round(float(r.get('vy', 0.0)), 3)
            cur['wz'] = round(float(r.get('wz', 0.0)), 3)
            cur['navigation_state'] = r.get('navigation_state', r.get('fsm', 'IDLE'))
            cur['last_telemetry_at'] = r.get('last_telemetry_at')
            battery_reported = r.get('battery_reported') is True
            battery = r.get('battery')
            cur['battery_reported'] = battery_reported and isinstance(battery, (int, float))
            cur['battery'] = round(float(battery), 2) if cur['battery_reported'] else None
            if tick % 10 == 0:
                cur['stats'] = {k: round(v, 1) if isinstance(v, float) else v for k, v in r['stats'].items()}
            cur['path'] = r['path']
            prev = prev_sent.get(rid, {})
            diff = {k: v for k, v in cur.items() if prev.get(k) != v}
            if diff:
                out[rid] = diff
            prev_sent[rid] = cur
        return out

    @profile_sync
    def make_patch(self) -> dict[str, Any]:
        S = self.engine.state
        patch: dict[str, Any] = {'sim': S['sim'], 'robots': self._robot_patch()}
        for sec in self.SECTIONS:
            prev = self._prev.get(sec, {})
            cur = {k: json.dumps(v, separators=(',', ':'), sort_keys=True) for k, v in S[sec].items()}
            diff = {k: S[sec][k] for k, raw in cur.items() if prev.get(k) != raw}
            for key in prev:
                if key not in cur:
                    diff[key] = None
            if diff:
                patch[sec] = diff
            self._prev[sec] = cur
        subsys = json.dumps(S['subsystems'], sort_keys=True)
        if subsys != self._prev_subsys:
            patch['subsystems'] = S['subsystems']
            self._prev_subsys = subsys
        if S['sim']['tick'] % SIM['KPI_EVERY'] == 0:
            patch['kpi'] = S['kpi']
        if S['recent_decisions'] and S['recent_decisions'][0]['id'] != self._sent_decision:
            self._sent_decision = S['recent_decisions'][0]['id']
            patch['recent_decisions'] = S['recent_decisions'][:20]
        return patch

    @profile_sync
    def heatmap_layer(self, kind: str, src: list[float], floor: int = 1) -> dict[str, Any]:
        g = self.engine.grid
        stride = 2
        cols = (g.cols + stride - 1) // stride
        rows = (g.rows + stride - 1) // stride
        values = [0.0] * (cols * rows)
        for r in range(g.rows):
            base = r * g.cols
            rr = (r // stride) * cols
            for c in range(g.cols):
                val = src[base + c]
                if val > 0:
                    values[rr + c // stride] += val
        mx = max(values) or 1.0
        return {'kind': kind, 'floor': floor, 'cols': cols, 'rows': rows, 'values': [round(x / mx, 2) for x in values], 'window_ticks': 200 if kind == 'TRAFFIC' else 6000}

    def reset(self, seed: int | None = None) -> None:
        if seed is not None:
            self.seed = seed
        self.engine = SimEngine(self.layout, seed=self.seed)
        self.engine.external_scheduler = True
        self.plc = PLCSimulator(self.layout)
        self.engine.state['conveyors'] = self.plc.snapshot()
        self._prepare_external_cache()
        self.last_telemetry_at = None
        self.last_telemetry_iso = None
        self.ros_bridge_connected = False
        self.connected_robot_ids.clear()
        self.robot_bridge_heartbeats.clear()
        self.bridge_status = 'DISCONNECTED' if self.is_external else 'LOCAL'
        self.nav2_state = 'OFFLINE' if self.is_external else 'LOCAL'
        self.operation_mode = 'IDLE' if self.is_external else 'SIMULATION'
        self.ros_diagnostics = {
            'ros': False, 'gazebo': False, 'controller_manager': False,
            'slam': False, 'nav2': False, 'tf': False, 'lidar': False,
            'nodes': [], 'topics': [], 'controllers': [],
            'simulation_time': None, 'last_update_at': None,
        }
        self.run_id = uuid.uuid4().hex[:8]
        self._prev = {}
        self._prev_subsys = ''
        self._sent_decision = ''
        self.last_sent_tick = 0
        self._snapshot_prev()

    def replace_layout(self, layout: dict[str, Any], *, pause: bool = True) -> None:
        """Replace the authoritative geometry used by the mock runtime."""
        self.layout = json.loads(json.dumps(layout))
        if pause:
            self.paused = True
        self.reset(self.seed)

    def publish_layout(self, body: dict[str, Any]) -> int:
        version = int(time.time())
        payload = json.dumps(body, ensure_ascii=False, indent=2)
        (self.layout_dir / f'layout-v{version}.json').write_text(payload, encoding='utf-8')
        (self.layout_dir / 'active.json').write_text(payload, encoding='utf-8')
        (self.layout_dir / 'draft.json').write_text(payload, encoding='utf-8')
        self.paused = True
        self.layout = json.loads(payload)
        self.engine = SimEngine(self.layout, seed=self.seed)
        self.engine.external_scheduler = True
        self.plc = PLCSimulator(self.layout)
        self.engine.state['conveyors'] = self.plc.snapshot()
        self.run_id = uuid.uuid4().hex[:8]
        self._snapshot_prev()
        self.last_sent_tick = 0
        return version

    def conveyor_snapshot(self) -> dict[str, dict[str, Any]]:
        return self.plc.snapshot()

    async def command_conveyor(self, conveyor_id: str, action: str, **kwargs: Any) -> dict[str, Any]:
        result = self.plc.command(conveyor_id, action, **kwargs)
        self.engine.state['conveyors'] = self.plc.snapshot()
        await self.broadcast_full()
        return result

    async def conveyor_handshake(self, conveyor_id: str, robot_id: str, item_id: str, phase: str) -> dict[str, Any]:
        result = self.plc.request_handshake(conveyor_id, robot_id, item_id, phase)
        self.engine.state['conveyors'] = self.plc.snapshot()
        await self.broadcast_full()
        return result

    async def run_whatif_safe(self, req: dict[str, Any]) -> dict[str, Any]:
        async with self._whatif_lock:
            start_tick = self.engine.state['sim']['tick']
            base, scen = self.engine.clone(), self.engine.clone()
            # What-if remains a self-contained simulation. The live/mock runtime
            # uses the persistent DB scheduler, but What-if needs the original
            # autonomous task generator/assignment logic to compare scenarios.
            base.external_scheduler = False
            scen.external_scheduler = False
            return await asyncio.to_thread(run_whatif, base, scen, req, start_tick)

    async def handle_message(self, consumer, data: dict[str, Any], user) -> None:
        route_monotonic = time.monotonic()
        consumer_monotonic = data.get('_consumer_monotonic')
        view_received_ms = data.get('_view_received_ms')
        data = {key: value for key, value in data.items() if key not in ('_consumer_monotonic', '_view_received_ms')}
        try:
            msg = client_adapter.validate_python(data)
        except ValidationError as exc:
            await consumer.send_json({'type': 'ERROR', 'code': 'BAD_MESSAGE', 'message': str(exc)[:300]})
            return
        t = msg.type
        eng = self.engine
        is_admin = getattr(user, 'role', None) == 'admin'

        if t == 'RESYNC':
            await consumer.send_json(self.full_message())
        elif t == 'SIM_CONTROL':
            if self.is_external:
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'BAD_MESSAGE',
                    'message': f'SIM_CONTROL is disabled in {self.runtime_mode}; the robot runtime is authoritative',
                })
                return
            if msg.action == 'PLAY':
                self.paused = False
                if msg.speed:
                    self.speed = msg.speed
                if self.speed == 0:
                    self.speed = 1
            elif msg.action == 'PAUSE':
                self.paused = True
            elif msg.action == 'RESET':
                self.reset()
                await self.broadcast(self.full_message())
                return
            if msg.speed is not None and msg.action != 'RESET':
                self.speed = msg.speed
                self.paused = self.speed == 0
            eng.state['sim']['speed'] = self.speed
            eng.state['sim']['mode'] = 'PAUSED' if self.paused else 'LIVE'
            await self.broadcast({'type': 'PATCH', 'base_tick': eng.state['sim']['tick'], 'tick': eng.state['sim']['tick'], 'patch': {'sim': eng.state['sim']}, 'events': []})
        elif t == 'INJECT':
            if not is_admin:
                await consumer.send_json({'type': 'ERROR', 'code': 'FORBIDDEN', 'message': 'Admin privileges required for scenario injection'})
                return
            eng.inject(msg.injection.model_dump(exclude_none=True))
            await self.broadcast_full()
        elif t == 'CLEAR_INJECTION':
            if not is_admin:
                await consumer.send_json({'type': 'ERROR', 'code': 'FORBIDDEN', 'message': 'Admin privileges required for scenario injection'})
                return
            eng.clear_injection(msg.kind, msg.target_id)
            await self.broadcast_full()
        elif t == 'CREATE_TASK':
            if not is_admin:
                await consumer.send_json({'type': 'ERROR', 'code': 'FORBIDDEN', 'message': 'Admin privileges required for task management'})
                return
            nt = msg.task
            try:
                task = eng.create_task(nt.type, nt.priority, nt.source, nt.destination, nt.load_units)
                if nt.deadline_s is not None:
                    task['deadline_tick'] = eng.state['sim']['tick'] + int(nt.deadline_s * 10)
                if nt.robot_id is not None:
                    try:
                        eng.assign_task(task['id'], nt.robot_id, source='USER')
                    except ValueError:
                        eng.state['tasks'].pop(task['id'], None)
                        raise
            except ValueError as exc:
                await consumer.send_json({'type': 'ERROR', 'code': 'BAD_TASK', 'message': str(exc)})
                return
            await self.broadcast_full()
        elif t == 'ASSIGN_TASK':
            if not is_admin:
                await consumer.send_json({'type': 'ERROR', 'code': 'FORBIDDEN', 'message': 'Admin privileges required for task management'})
                return
            try:
                eng.assign_task(msg.task_id, msg.robot_id, source='USER')
            except ValueError as exc:
                await consumer.send_json({'type': 'ERROR', 'code': 'BAD_ASSIGN', 'message': str(exc)})
                return
            await self.broadcast_full()
        elif t == 'ACK_ALERT':
            eng.ack_alert(msg.alert_id)
            await self.broadcast_full()
        elif t == 'SELECT_ROBOT':
            return
        elif t == 'ROBOT_MODE':
            if (msg.robot_id in self.local_map_transitions
                    and msg.mode != 'MANUAL'):
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'LOCAL_MAP_TRANSITION',
                    'message': 'only MANUAL mode is allowed while the saved local map is loading',
                })
                return
            if not self.is_external:
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': 'Manual robot control requires an active ROS robot bridge',
                })
                return
            if not self.robot_bridge_online(msg.robot_id):
                await consumer.send_json({
                    'type': 'ROBOT_CONTROL_STATUS', 'robot_id': msg.robot_id,
                    'mode': msg.mode, 'accepted': False,
                    'reason': 'ROS bridge for this robot is offline',
                })
                return
            import uuid
            request_id = uuid.uuid4().hex
            if not hasattr(self, 'control_mode_requests'):
                self.control_mode_requests = {}
            self.control_mode_requests[msg.robot_id] = {
                'request_id': request_id, 'requested_mode': msg.mode,
                'applied_mode': eng.state['robots'].get(msg.robot_id, {}).get('control_mode'),
                'mode_transition_state': 'REQUESTED',
            }
            result = await self.gateway().send_command(
                msg.robot_id, 'CONTROL_MODE', {'mode': msg.mode, 'request_id': request_id})
            if msg.mode == 'MANUAL' and result.get('ok'):
                self.invalidate_path_previews(msg.robot_id, 'control mode changed')
            await consumer.send_json({
                'type': 'ROBOT_CONTROL_STATUS', 'robot_id': msg.robot_id,
                'mode': msg.mode, 'accepted': bool(result.get('ok')),
                'requested_mode': msg.mode, 'request_id': request_id,
                'mode_transition_state': 'REQUESTED' if result.get('ok') else 'FAILED',
                'reason': None if result.get('ok') else 'ROS bridge is offline',
            })
        elif t == 'ROBOT_MANUAL':
            if (msg.robot_id in self.local_map_transitions
                    and msg.action != 'STOP'):
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'LOCAL_MAP_TRANSITION',
                    'message': 'manual motion is blocked while the saved local map is loading',
                })
                return
            if not self.is_external:
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': 'Manual robot control requires an active ROS robot bridge',
                })
                return
            if not self.robot_bridge_online(msg.robot_id):
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': f'ROS bridge for {msg.robot_id} is offline; manual command was not sent',
                })
                return
            if not hasattr(self, 'manual_owners'):
                self.manual_owners = {}
            if msg.action != 'STOP':
                self.manual_owners[msg.robot_id] = consumer.channel_name
            else:
                self.manual_owners.pop(msg.robot_id, None)
            manual_payload = {'action': msg.action}
            if msg.sequence_id is not None:
                manual_payload['sequence_id'] = msg.sequence_id
            if os.environ.get('WARETWIN_MANUAL_TIMING') == '1':
                manual_payload['_timing'] = {'T0': msg.client_monotonic,
                    'T1': consumer_monotonic, 'T2': route_monotonic}
            result = await self.gateway().send_command(
                msg.robot_id, 'MANUAL_CMD', manual_payload)
            if not result.get('ok'):
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': 'ROS bridge is offline; manual command was not sent',
                })
        elif t == 'ROBOT_DETAIL_VIEW':
            log.info('DETAIL_VIEW_TIMING request=%s view=%s received_ms=%.3f online=%s',
                msg.request_id, msg.view, view_received_ms or time.time() * 1000,
                self.robot_bridge_online(msg.robot_id))
            if self.is_external and self.robot_bridge_online(msg.robot_id):
                result = await self.gateway().send_command(msg.robot_id, 'DETAIL_VIEW', {
                    'view': msg.view,
                    'request_id': msg.request_id,
                    'django_received_ms': view_received_ms or time.time() * 1000,
                })
                if not result.get('ok'):
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                        'message': 'robot detail visualization mode was not sent to the ROS bridge',
                    })
            else:
                await consumer.send_json({'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': 'robot detail visualization requires an online ROS bridge'})
        elif t == 'PATH_PREVIEW_INVALIDATE':
            self.invalidate_path_previews(msg.robot_id, 'navigation target changed')
        elif t == 'PATH_PREVIEW_REQUEST':
            now = time.monotonic()
            request_id = str(getattr(msg, 'request_id', '') or '')
            active = self.active_map_state(msg.robot_id)
            source_type = str(msg.source_type or 'MAP_POINT').upper()
            source_id = str(msg.tag_id) if source_type == 'TAG' and msg.tag_id is not None else None
            resolved_target = None
            target_metadata: dict[str, Any] = {}
            tag_revision = None
            registry_revision = None

            async def preview_failure(status, reason):
                goal = ({'x': resolved_target['x'], 'y': resolved_target['y'],
                         'yaw': resolved_target['yaw'], 'frame_id': 'map'}
                        if resolved_target is not None else None)
                await consumer.send_json({
                    'type': 'PATH_PREVIEW_RESULT', 'robot_id': msg.robot_id,
                    'request_id': request_id, 'status': status, 'reason': reason,
                    'source_type': source_type, 'source_id': source_id,
                    'tag_id': msg.tag_id if source_type == 'TAG' else None,
                    'tag_revision': tag_revision, 'registry_revision': registry_revision,
                    'path': [], 'path_length_m': None, 'goal': goal,
                    'active_map_id': active.get('active_map_id'),
                    'active_map_revision': active.get('active_map_revision'),
                    'canonical_map_revision': active.get('canonical_map_revision'),
                    'timestamp': datetime.now(timezone.utc).isoformat(),
                })

            if msg.robot_id in self.local_map_transitions:
                await preview_failure('INVALID', 'a local Nav2 map transition is in progress')
                return
            robot_mode = str(self.robot_runtime_modes.get(msg.robot_id) or self.operation_mode).upper()
            if robot_mode not in NAVIGATION_CAPABLE_MODES:
                await preview_failure('INVALID', 'Nav2 path preview is unavailable in the current stack runtime')
                return
            blocker = self.unified_navigation_blocker(msg.robot_id)
            if blocker:
                await preview_failure('INVALID', blocker)
                return
            if not self.is_external or not self.robot_bridge_online(msg.robot_id):
                await preview_failure('NO_PATH', 'robot ROS bridge is offline')
                return
            if not active.get('active_map_id') or not active.get('active_map_revision'):
                await preview_failure('INVALID', 'the robot active map is not confirmed')
                return
            requested_map_id = str(getattr(msg, 'map_id', '') or getattr(msg, 'active_map_id', '') or '')
            requested_map_revision = str(getattr(msg, 'map_revision', '')
                                         or getattr(msg, 'active_map_revision', '') or '')
            if (str(getattr(msg, 'active_map_id', '') or '') != active['active_map_id']
                    or str(getattr(msg, 'active_map_revision', '') or '') != active['active_map_revision']
                    or requested_map_id != active['active_map_id']
                    or requested_map_revision != active['active_map_revision']):
                await preview_failure('INVALID', 'PATH_PREVIEW_MAP_MISMATCH: active map changed')
                return
            if active['map_sync_status'] not in ('CANONICAL', 'LOCAL_ONLY'):
                await preview_failure('INVALID', f'map is not ready for local navigation ({active["map_sync_status"]})')
                return
            localization_state = self.navigation_localization_state(msg.robot_id, active)
            if localization_state is None:
                await preview_failure('INVALID', 'a fresh TF pose in the confirmed active map is required for navigation')
                return
            try:
                resolved_target = await self.resolve_navigation_target(
                    robot_id=msg.robot_id,
                    source_type=source_type,
                    active_map=active,
                    tag_id=msg.tag_id,
                    x=msg.x,
                    y=msg.y,
                    yaw=msg.yaw,
                )
            except Exception as exc:
                code = getattr(exc, 'code', 'NAVIGATION_TARGET_INVALID')
                await preview_failure('INVALID', f'{code}: {exc}')
                return
            target = {axis: float(resolved_target[axis]) for axis in ('x', 'y', 'yaw')}
            target_metadata = dict(resolved_target.get('metadata') or {})
            if source_type == 'TAG':
                tag_revision = target_metadata.get('tag_revision')
                registry_revision = target_metadata.get('registry_revision')
                if (not msg.tag_revision or not msg.registry_revision
                        or str(msg.tag_revision) != str(tag_revision)
                        or str(msg.registry_revision) != str(registry_revision)):
                    await preview_failure('INVALID', 'TAG_REVISION_MISMATCH: Tag or registry data changed; select the Tag again')
                    return
                source_id = str(resolved_target.get('source_id'))

            for key, pending in list(self.path_preview_requests.items()):
                if now - float(pending.get('created_monotonic', now)) > 120.0:
                    self.path_preview_requests.pop(key, None)
                    self.expired_path_previews[key] = now
            for key, preview in list(self.path_preview_results.items()):
                if now - float(preview.get('created_monotonic', now)) > 120.0:
                    self.path_preview_results.pop(key, None)
                    self.approved_path_previews.pop(key, None)
                    self.expired_path_previews[key] = now
            for key, marked_at in list(self.expired_path_previews.items()):
                if now - marked_at > 120.0:
                    self.expired_path_previews.pop(key, None)
                    self.path_preview_invalidations.pop(key, None)

            key = (msg.robot_id, request_id)
            self.path_preview_invalidations.pop(key, None)
            self.expired_path_previews.pop(key, None)
            request = {
                'goal': target,
                'source_type': source_type,
                'source_id': source_id,
                'tag_id': msg.tag_id if source_type == 'TAG' else None,
                'tag_revision': tag_revision,
                'registry_revision': registry_revision,
                'target_metadata': target_metadata,
                'active_map_id': active['active_map_id'],
                'active_map_revision': active['active_map_revision'],
                'canonical_map_revision': active['canonical_map_revision'],
                'localization_state': localization_state,
                'created_monotonic': now,
            }
            self.path_preview_requests[key] = request
            result = await self.gateway().send_command(msg.robot_id, 'PATH_PREVIEW', {
                'request_id': request_id, **target, 'frame_id': msg.frame_id,
                'source_type': source_type, 'source_id': source_id,
                'tag_id': msg.tag_id if source_type == 'TAG' else None,
                'tag_revision': tag_revision, 'registry_revision': registry_revision,
                'active_map_id': active['active_map_id'],
                'active_map_revision': active['active_map_revision'],
                'canonical_map_revision': active['canonical_map_revision'],
            })
            if not result.get('ok'):
                self.path_preview_requests.pop(key, None)
                await preview_failure('NO_PATH', 'path preview command could not reach the ROS bridge')
        elif t in ('NAV_GOAL', 'NAV_CANCEL', 'NAV_PAUSE', 'NAV_RESUME'):
            if t == 'NAV_GOAL' and not getattr(msg, 'preview_request_id', None):
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'PATH_PREVIEW_REQUIRED',
                    'message': 'Send Goal requires a valid Nav2 path preview ID',
                })
                return
            if not self.is_external:
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': 'Autonomous robot control requires an active ROS robot bridge',
                })
                return
            if not self.robot_bridge_online(msg.robot_id):
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': f'ROS bridge for {msg.robot_id} is offline; navigation command was not sent',
                })
                return
            if t in ('NAV_GOAL', 'NAV_RESUME') and msg.robot_id in self.local_map_transitions:
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'LOCAL_MAP_TRANSITION',
                    'message': 'navigation is blocked while the selected robot map is loading',
                })
                return
            if t in ('NAV_GOAL', 'NAV_RESUME'):
                robot_mode = str(self.robot_runtime_modes.get(msg.robot_id) or self.operation_mode).upper()
                if robot_mode not in NAVIGATION_CAPABLE_MODES:
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'NAVIGATION_UNAVAILABLE',
                        'message': 'Nav2 goals are unavailable in the current stack runtime',
                    })
                    return
                blocker = self.unified_navigation_blocker(msg.robot_id)
                if blocker:
                    await consumer.send_json({
                        'type': 'ERROR', 'code': blocker.split(':', 1)[0],
                        'message': blocker,
                    })
                    return
            if t == 'NAV_GOAL':
                if msg.frame_id != 'map':
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'INVALID_GOAL_FRAME',
                        'message': 'Navigation goals must use frame_id=map',
                    })
                    return
                active = self.active_map_state(msg.robot_id)
                if (not active.get('active_map_id') or not active.get('active_map_revision')
                        or str(getattr(msg, 'active_map_id', '') or '') != active['active_map_id']
                        or str(getattr(msg, 'active_map_revision', '') or '') != active['active_map_revision']
                        or (getattr(msg, 'map_id', None) is not None
                            and str(msg.map_id) != active['active_map_id'])
                        or (getattr(msg, 'map_revision', None) is not None
                            and str(msg.map_revision) != active['active_map_revision'])):
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'PATH_PREVIEW_MAP_MISMATCH',
                        'message': 'the requested active map does not match the robot runtime map',
                    })
                    return
                if active['map_sync_status'] not in ('CANONICAL', 'LOCAL_ONLY'):
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'MAP_OUT_OF_SYNC',
                        'message': f'local navigation is blocked because the active map is not ready ({active["map_sync_status"]})',
                    })
                    return

                preview_id = str(msg.preview_request_id)
                key = (msg.robot_id, preview_id)
                preview = self.path_preview_results.get(key)
                if preview is None:
                    if key in self.expired_path_previews:
                        code, message = 'PATH_PREVIEW_EXPIRED', 'the approved Nav2 path preview has expired'
                    elif key in self.path_preview_invalidations:
                        reason = self.path_preview_invalidations[key]
                        code = 'PATH_PREVIEW_MAP_MISMATCH' if 'map' in reason else 'PATH_PREVIEW_INVALID'
                        message = reason
                    elif any(candidate[0] != msg.robot_id and candidate[1] == preview_id
                             for candidate in self.path_preview_results):
                        code, message = 'PATH_PREVIEW_INVALID', 'path preview belongs to a different robot'
                    else:
                        code, message = 'PATH_PREVIEW_INVALID', 'path preview is unknown or was not approved'
                    await consumer.send_json({'type': 'ERROR', 'code': code, 'message': message})
                    return
                if time.monotonic() - float(preview.get('created_monotonic', 0.0)) > 120.0:
                    self.path_preview_results.pop(key, None)
                    self.approved_path_previews.pop(key, None)
                    self.expired_path_previews[key] = time.monotonic()
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'PATH_PREVIEW_EXPIRED',
                        'message': 'the approved Nav2 path preview has expired',
                    })
                    return
                if preview.get('status') == 'INVALID':
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'PATH_PREVIEW_INVALID',
                        'message': 'the Nav2 path preview result was invalid',
                    })
                    return
                if not preview.get('path_found') or key not in self.approved_path_previews:
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'NO_VALID_PATH',
                        'message': 'Nav2 did not return a valid path for this preview',
                    })
                    return
                if (preview.get('active_map_id') != active['active_map_id']
                        or preview.get('active_map_revision') != active['active_map_revision']
                        or preview.get('localization_state') != self.navigation_localization_state(msg.robot_id, active)
                        or str(getattr(msg, 'active_map_id', '') or '') != preview.get('active_map_id')
                        or str(getattr(msg, 'active_map_revision', '') or '') != preview.get('active_map_revision')):
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'PATH_PREVIEW_MAP_MISMATCH',
                        'message': 'the active map or revision changed after path preview',
                    })
                    return
                target = preview.get('goal') or {}
                try:
                    matches_target = all(
                        math.isfinite(float(target[axis]))
                        and math.isfinite(float(getattr(msg, axis)))
                        and abs(float(target[axis]) - float(getattr(msg, axis))) <= 1e-4
                        for axis in ('x', 'y', 'yaw')
                    )
                except (KeyError, TypeError, ValueError):
                    matches_target = False
                if not matches_target:
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'PATH_PREVIEW_INVALID',
                        'message': 'goal does not match the approved Nav2 path preview',
                    })
                    return
                source_type = str(preview.get('source_type') or 'MAP_POINT')
                source_id = preview.get('source_id')
                requested_source_type = str(getattr(msg, 'source_type', '') or '')
                source_mismatch = (
                    (source_type == 'TAG' and (requested_source_type != 'TAG'
                     or str(getattr(msg, 'source_id', '') or '') != str(source_id)))
                    or (source_type == 'MAP_POINT' and requested_source_type not in ('', 'MAP_POINT'))
                )
                if source_mismatch:
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'PATH_PREVIEW_INVALID',
                        'message': 'navigation target source does not match the approved path preview',
                    })
                    return
                if source_type == 'TAG':
                    try:
                        resolved_now = await self.resolve_navigation_target(
                            robot_id=msg.robot_id, source_type='TAG', active_map=active,
                            tag_id=preview.get('tag_id'),
                        )
                        current_metadata = resolved_now.get('metadata') or {}
                        current_tag_matches = (
                            str(resolved_now.get('source_id')) == str(source_id)
                            and current_metadata.get('tag_revision') == preview.get('tag_revision')
                            and current_metadata.get('registry_revision') == preview.get('registry_revision')
                            and all(abs(float(resolved_now[axis]) - float(target[axis])) <= 1e-4
                                    for axis in ('x', 'y', 'yaw'))
                        )
                    except Exception:
                        current_tag_matches = False
                    if not current_tag_matches:
                        await consumer.send_json({
                            'type': 'ERROR', 'code': 'PATH_PREVIEW_INVALID',
                            'message': 'Tag or registry data changed after path preview; request a new preview',
                        })
                        return
                self.approved_path_previews.pop(key, None)
                self.path_preview_results.pop(key, None)
                result = await self.gateway().send_command(msg.robot_id, 'NAVIGATE', {
                    'x': target['x'], 'y': target['y'], 'yaw': target['yaw'], 'frame_id': msg.frame_id,
                    'source_type': source_type, 'source_id': source_id,
                    'tag_id': preview.get('tag_id'),
                    'tag_revision': preview.get('tag_revision'),
                    'registry_revision': preview.get('registry_revision'),
                    'preview_request_id': preview_id,
                    'active_map_id': active['active_map_id'],
                    'active_map_revision': active['active_map_revision'],
                    'canonical_map_revision': active['canonical_map_revision'],
                })
                if result.get('ok'):
                    await self.broadcast({'type': 'NAV_GOAL', 'goal': {
                        'robot_id': msg.robot_id, 'frame_id': msg.frame_id,
                        'x': target['x'], 'y': target['y'], 'yaw': target['yaw'],
                        'source_type': source_type, 'source_id': source_id,
                        'active_map_id': active['active_map_id'],
                        'active_map_revision': active['active_map_revision'],
                        'status': 'SENT', 'timestamp': datetime.now(timezone.utc).isoformat(),
                    }})
            else:
                active = self.active_map_state(msg.robot_id)
                if (t == 'NAV_RESUME'
                        and active.get('map_sync_status') not in ('CANONICAL', 'LOCAL_ONLY')):
                    await consumer.send_json({
                        'type': 'ERROR', 'code': 'MAP_OUT_OF_SYNC',
                        'message': f'navigation resume is blocked by active map state {active["map_sync_status"]}',
                    })
                    return
                result = await self.gateway().send_command(msg.robot_id, t, {})
            if not result.get('ok'):
                await consumer.send_json({
                    'type': 'ERROR', 'code': 'CONTROL_UNAVAILABLE',
                    'message': 'ROS bridge is offline; navigation command was not sent',
                })
        elif t == 'COPILOT_ASK':
            snapshot = json.loads(json.dumps(eng.state))
            reply = await asyncio.to_thread(copilot_ai.answer, msg.question, snapshot, self.layout)
            cites = []
            for c in reply.get('citations', []):
                if c.startswith('E'):
                    cites.append({'event_id': c})
                elif c.startswith('R') and len(c) == 3:
                    cites.append({'robot_id': c})
                elif c.startswith('A') and len(c) == 5:
                    cites.append({'task_id': c})
            eng.emit('AI_DECISION', 'AI_AGENT', 'INFO', f"Copilot answered: {msg.question[:60]}", payload={'model': reply.get('model')})
            await consumer.send_json({'type': 'COPILOT_REPLY', 'request_id': msg.request_id, 'text': reply['text'], 'citations': cites, 'model': reply.get('model')})
        elif t == 'WHATIF_RUN':
            req = msg.request.model_dump(exclude_none=True)
            result = await self.run_whatif_safe(req)
            await consumer.send_json({'type': 'WHATIF_RESULT', 'request_id': msg.request_id, 'result': result})

    def validate_state(self) -> None:
        TwinState.model_validate(self.engine.state)


runtime = TwinRuntime()
