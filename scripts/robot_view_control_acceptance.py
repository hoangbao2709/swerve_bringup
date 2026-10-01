#!/usr/bin/env python3
"""Real browser view switches with independent Web manual refresh / ROS probe."""
import subprocess
import os
from pathlib import Path

# Reuse the verified observer/sender/settling setup; no direct ROS motion.
os.environ['ACCEPTANCE_LATEST_FEEDBACK'] = '1'
os.environ['ACCEPTANCE_MANUAL_PROCESS'] = '1'
exec(compile(Path('scripts/local_control_teleop_acceptance.py').read_text().split('results={')[0],
    'scripts/local_control_teleop_acceptance.py', 'exec'))
from std_msgs.msg import Bool
n.create_subscription(Bool, '/emergency_stop', lambda m: record('estop', bool(m.data)), 10)
ws.send(json.dumps({'type': 'ROBOT_DETAIL_VIEW', 'robot_id': 'R01', 'view': 'GLOBAL',
    'request_id': 'safety-probe-initial', 'delivery_ack': True}))
results = {}
process = None
view_stop = threading.Event()
view_thread = None


def wait(predicate, timeout=15):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate(): return True
        time.sleep(.01)
    return bool(predicate())


def mode(value):
    start = time.monotonic()
    ws.send(json.dumps({'type': 'ROBOT_MODE', 'robot_id': 'R01', 'mode': value}))
    assert wait(lambda: any(t >= start and message.get('mode_transition_state') == 'APPLIED'
        and message.get('applied_mode') == value for t, message in list(statuses))
        and data['arbiter'][-1][1]['active_control_mode'] == value), 'mode did not apply'
    return start


def begin():
    mode('MANUAL')
    rest = settle(); assert rest['passed'], json.dumps(rest)
    start = time.monotonic(); command('ROTATE_LEFT')
    assert wait(lambda: data['arbiter'][-1][1]['active_command_source'] == 'WEB_MANUAL'
        and any(row[0] >= start and abs(row[1][2]) > .1 for row in list(data['/cmd_vel_manual']))
        and any(row[0] >= start and abs(row[1][2]) > .1 for row in list(data['/cmd_vel_selected']))
        and abs(data['gazebo'][-1][1]['velocity'][2]) > .05), 'real manual rotation did not acquire'


def stopped(start, estop=False):
    assert wait(lambda: data['/cmd_vel_selected'][-1][0] >= start
        and max(map(abs, data['/cmd_vel_selected'][-1][1])) <= 1e-6), 'selected zero missing'
    first = next(row for row in list(data['/cmd_vel_selected'])
        if row[0] >= start and max(map(abs, row[1])) <= 1e-6)
    rest = settle()
    owner = data['arbiter'][-1][1]['active_command_source']
    return {'passed': rest['passed'] and owner == ('ESTOP' if estop else 'NONE'),
        'zero_latency_ms': (first[0] - start) * 1000, 'settling': rest, 'owner': owner}


try:
    assert wait(lambda: sim[0] > 0 and all(data[key] for key in ('arbiter', 'gazebo', 'odom', 'joints', '/cmd_vel_selected')))
    assert not data['arbiter'][-1][1]['estop_active'], 'E-STOP must be known clear before motion'
    begin()
    start, sim_start = time.monotonic(), sim[0]
    pose_start = data['gazebo'][-1][1]['pose']
    process = subprocess.Popen(['node', 'scripts/robot_view_performance.cjs'], env=dict(os.environ,
        VIEW_SWITCH_COUNT=os.environ.get('VIEW_CONTROL_COUNT', '10'),
        VIEW_SAMPLE_FPS='0' if os.environ.get('VIEW_CONTROL_COUNT') else '1',
        VIEW_PERFORMANCE_OUTPUT='.runtime/view-performance-after.json'))
    while process.poll() is None:
        if time.monotonic() - start > 150 or sim[0] - sim_start > 20:
            raise RuntimeError('safe view acceptance wall/simulation watchdog')
        if a.dist(pose_start, data['gazebo'][-1][1]['pose']) > .4:
            raise RuntimeError('safe in-place rotation displacement limit')
        if sender.error: raise sender.error
        time.sleep(.03)
    end = time.monotonic()
    command('STOP')
    results['browser_completed'] = {'passed': process.returncode == 0}
    performance = json.loads(Path('.runtime/view-performance-after.json').read_text())
    results['view_performance'] = {'passed': performance.get('passed') is True}
    rows = [row for row in list(data['/cmd_vel_manual']) if start <= row[0] <= end]
    owners = [row[1] for row in list(data['arbiter']) if start <= row[0] <= end]
    max_gap = max((b[0] - arow[0]) * 1000 for arow, b in zip(rows, rows[1:]))
    results['manual_continuity'] = {'passed': len(rows) > 20 and max_gap <= 200
        and all(max(map(abs, row[1])) > .1 for row in rows)
        and all(owner['active_command_source'] == 'WEB_MANUAL' for owner in owners),
        'MAX_CMD_VEL_MANUAL_GAP_MS': max_gap, 'samples': len(rows),
        'zero_manual_samples': sum(max(map(abs, row[1])) <= .1 for row in rows),
        'owner_counts': {owner: sum(item['active_command_source'] == owner for item in owners)
            for owner in set(item['active_command_source'] for item in owners)},
        'wall_seconds': end - start, 'simulation_seconds': sim[0] - sim_start,
        'gazebo_rtf': (sim[0] - sim_start) / (end - start)}
    results['STOP'] = stopped(end)
    assert results['STOP']['passed'], 'strict STOP settling failed; further motion safety checks blocked'

    def switch_views():
        index = 0
        views = ['GLOBAL', 'LIDAR_2D', 'LIDAR_3D', 'LIDAR_2D', 'GLOBAL', 'LIDAR_3D']
        while not view_stop.is_set():
            ws.send(json.dumps({'type': 'ROBOT_DETAIL_VIEW', 'robot_id': 'R01',
                'view': views[index % len(views)], 'request_id': f'safety-view-{index}',
                'delivery_ack': True}))
            index += 1; view_stop.wait(.15)
    view_thread = threading.Thread(target=switch_views, daemon=True); view_thread.start()
    begin(); sender.hold(None)
    results['MODE_CHANGE'] = stopped(mode('AUTONOMOUS'))
    begin(); sender.hold(None)
    estop_token = a.authenticate(backend)
    triggered = time.monotonic()
    assert a.http_json(backend + '/api/robots/R01/emergency-stop', {}, token=estop_token)['ok']
    assert wait(lambda: data['arbiter'][-1][1]['estop_active'])
    receipt = next(row[0] for row in list(data['estop']) if row[0] >= triggered and row[1])
    results['E_STOP'] = stopped(triggered, estop=True)
    zero = next(row[0] for row in list(data['/cmd_vel_selected'])
        if row[0] >= receipt and max(map(abs, row[1])) <= 1e-6)
    results['E_STOP']['ros_receipt_zero_latency_ms'] = (zero - receipt) * 1000
    results['E_STOP']['passed'] &= results['E_STOP']['ros_receipt_zero_latency_ms'] <= 100
    clear_start = time.monotonic()
    assert a.http_json(backend + '/api/robots/R01/clear-emergency-stop', {}, token=a.authenticate(backend))['ok']
    assert wait(lambda: not data['arbiter'][-1][1]['estop_active'])
    time.sleep(2)
    results['E_STOP_CLEAR_NO_RESUME'] = {'passed': all(max(map(abs, row[1])) <= 1e-6
        for row in list(data['/cmd_vel_selected']) if row[0] >= clear_start)}
    results['passed'] = all(value['passed'] for value in results.values())
    print(json.dumps(results, indent=2), flush=True)
finally:
    view_stop.set()
    if view_thread: view_thread.join(3)
    if process is not None and process.poll() is None:
        process.terminate()
        try: process.wait(timeout=3)
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=3)
    try:
        sender.close()
        mode('AUTONOMOUS')
    finally:
        closed = True; ws.close(); rclpy.shutdown(); thread.join(3); n.destroy_node()
        Path('.runtime/view-control-safety.json').write_text(json.dumps(results, indent=2))
if not results.get('passed'): raise SystemExit(1)
