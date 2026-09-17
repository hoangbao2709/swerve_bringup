from __future__ import annotations

import asyncio
import json
import logging
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
from .coordinates import ros_pose_to_waretwin, ros_twist_to_waretwin

log = logging.getLogger(__name__)
TICK_S = SIM['TICK_S']
HEATMAP_EVERY = 30
client_adapter: TypeAdapter[Any] = TypeAdapter(ClientMessage)


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
    """In-process mock digital twin used until Robot Server / ROS2 is connected.

    The frontend contract is intentionally the same as the existing FastAPI backend:
    FULL/PATCH/HEATMAP and the current REST payloads. Later, SimEngine can be replaced
    by a live RobotGateway without rewriting the React UI.
    """

    SECTIONS = ('tasks', 'lifts', 'zones', 'conveyors', 'cameras', 'sensors', 'people', 'alerts')

    def __init__(self) -> None:
        self.layout_dir = Path(settings.BASE_DIR) / 'layouts'
        self.layout_dir.mkdir(exist_ok=True)
        active = self.layout_dir / 'active.json'
        if active.exists():
            try:
                self.layout = json.loads(active.read_text(encoding='utf-8'))
            except Exception:
                self.layout = load_layout()
        else:
            self.layout = load_layout()
        self.seed = int(os.getenv('TWIN_SEED', '42'))
        self.engine = SimEngine(self.layout, seed=self.seed)
        self.engine.external_scheduler = True
        self.plc = PLCSimulator(self.layout)
        self.engine.state['conveyors'] = self.plc.snapshot()
        self.runtime_mode = str(getattr(settings, 'WARETWIN_RUNTIME_MODE', 'LOCAL_SIM'))
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
        self.last_telemetry_at: float | None = None
        self.last_telemetry_iso: str | None = None
        self.last_ros_heartbeat: float | None = None
        self.nav2_state = 'OFFLINE' if self.runtime_mode != 'LOCAL_SIM' else 'LOCAL'
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
            robot['vx'] = robot['vy'] = robot['wz'] = 0.0
            robot['path'] = []
            robot['path_index'] = 0

    def gateway(self):
        if self.robot_gateway is None:
            from .gateways.ros_bridge import RosBridgeGateway
            self.robot_gateway = RosBridgeGateway()
        return self.robot_gateway

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

    async def after_ticks(self) -> None:
        S = self.engine.state
        S['sim']['speed'] = self.speed
        S['sim']['mode'] = 'LIVE' if self.is_external else ('PAUSED' if self.paused else 'LIVE')
        # Persistent scheduler is authoritative for simulated work. Run it at 1 Hz
        # simulation time so planned database routes are dispatched to the same
        # movement/A* engine that drives the 2D/3D robot models.
        if S['sim']['tick'] % 10 == 0:
            try:
                if self.is_external:
                    from .schedule_services import prepare_external_dispatches
                    goals, changed = await sync_to_async(
                        prepare_external_dispatches, thread_sensitive=True)(self)
                    for goal in goals:
                        result = await self.gateway().send_command(goal['robot_id'], 'NAVIGATE', goal)
                        if not result.get('ok'):
                            log.warning('ROS bridge unavailable for %s', goal.get('schedule_id'))
                            from .schedule_services import release_external_dispatch
                            await sync_to_async(release_external_dispatch, thread_sensitive=True)(
                                goal['schedule_id'], goal['stop_id'])
                else:
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
        return {
            'type': 'RUNTIME_STATUS', 'runtime_mode': self.runtime_mode,
            'ros_connected': self.ros_bridge_connected,
            'nav2_state': self.nav2_state,
            'last_telemetry_at': self.last_telemetry_iso,
        }

    async def bridge_connected(self) -> None:
        self.ros_bridge_connected = True
        self.nav2_state = 'CONNECTED'
        await self.broadcast_runtime_status()

    async def bridge_disconnected(self) -> None:
        self.ros_bridge_connected = False
        self.nav2_state = 'OFFLINE'
        if self.is_external:
            await self._mark_external_offline()
        await self.broadcast_runtime_status()

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
        if not rid:
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
        nav = str(data.get('navigation_state') or 'IDLE').upper()
        robot.update(pose, twist, {'navigation_state': nav, 'last_telemetry_at': now_iso,
                                   'status': 'ACTIVE', 'fsm': self._fsm_from_nav(nav)})
        self.last_telemetry_at = time.monotonic()
        self.last_ros_heartbeat = self.last_telemetry_at
        self.ros_bridge_connected = True
        self.last_telemetry_iso = now_iso
        if self.client_count:
            patch = self.make_patch()
            await self.broadcast({'type': 'PATCH', 'base_tick': self.last_sent_tick,
                                  'tick': self.engine.state['sim']['tick'], 'patch': patch, 'events': []})
            self.last_sent_tick = self.engine.state['sim']['tick']

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
        elif kind == 'HEARTBEAT':
            self.last_ros_heartbeat = time.monotonic()
            self.ros_bridge_connected = True
            self.nav2_state = str(data.get('nav2_state') or 'CONNECTED')
            await self.broadcast_runtime_status()

    def full_message(self) -> dict[str, Any]:
        S = self.engine.state
        S['sim']['speed'] = self.speed
        S['sim']['mode'] = 'LIVE' if self.is_external else ('PAUSED' if self.paused else 'LIVE')
        return {'type': 'FULL', 'state': S}

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
            cur['battery'] = round(r['battery'], 2)
            if tick % 10 == 0:
                cur['stats'] = {k: round(v, 1) if isinstance(v, float) else v for k, v in r['stats'].items()}
            cur['path'] = r['path']
            prev = prev_sent.get(rid, {})
            diff = {k: v for k, v in cur.items() if prev.get(k) != v}
            if diff:
                out[rid] = diff
            prev_sent[rid] = cur
        return out

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
        self.nav2_state = 'OFFLINE' if self.is_external else 'LOCAL'
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
                    'message': f'SIM_CONTROL is disabled in {self.runtime_mode}; ROS/Gazebo is authoritative',
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
