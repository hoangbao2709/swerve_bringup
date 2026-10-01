"""Physics-time settling monitor; no commands, synthetic feedback or ROS imports."""
import math
from collections import deque


class MechanicalSettling:
    def __init__(self, sim_start, wall_start, *, timeout_sim=45.0, timeout_wall=180.0,
                 clock_stall_wall=5.0, window_sim=0.8, body_tolerance=0.02,
                 wheel_position_rate=0.005, body_position_rate=0.001,
                 steering_span=0.03, data_age_wall=2.0, wheel_radius=None):
        if wheel_radius is not None:
            if not math.isfinite(wheel_radius) or wheel_radius <= 0:
                raise ValueError('measured wheel radius must be positive')
            # Geometry adds a physical bound and reporting; it must never
            # relax the established encoder-position settling limit.
            wheel_position_rate = min(wheel_position_rate, body_position_rate / wheel_radius)
        if not all(math.isfinite(value) and value > 0 for value in (
                timeout_sim, timeout_wall, clock_stall_wall, window_sim,
                body_tolerance, wheel_position_rate, body_position_rate,
                steering_span, data_age_wall)):
            raise ValueError('settling limits must be positive')
        self.sim_start = self.last_sim = sim_start
        self.wall_start = self.last_advance_wall = wall_start
        self.timeout_sim, self.timeout_wall = timeout_sim, timeout_wall
        self.clock_stall_wall, self.window_sim = clock_stall_wall, window_sim
        self.body_tolerance, self.wheel_position_rate = body_tolerance, wheel_position_rate
        self.body_position_rate, self.steering_span = body_position_rate, steering_span
        self.data_age_wall = data_age_wall
        self.wheel_radius = wheel_radius
        self.samples = deque(maxlen=2000)
        self.metrics = {}

    def timing(self, sim, wall):
        elapsed_sim, elapsed_wall = sim - self.sim_start, wall - self.wall_start
        return {'settling_sim_seconds': elapsed_sim, 'settling_wall_seconds': elapsed_wall,
                'gazebo_rtf': elapsed_sim / elapsed_wall if elapsed_wall > 0 else None}

    def update(self, sim, wall, sample):
        """Return None while waiting; sample contains only observed runtime values."""
        reason = None
        if sim < self.last_sim:
            reason = 'SIM_CLOCK_REWOUND'
        elif sim > self.last_sim:
            self.last_sim, self.last_advance_wall = sim, wall
        elif wall - self.last_advance_wall >= self.clock_stall_wall:
            reason = 'SIM_CLOCK_STALLED'
        if reason is None and sim - self.sim_start >= self.timeout_sim:
            reason = 'MECHANICAL_SETTLING_SIM_TIMEOUT'
        if reason is None and wall - self.wall_start >= self.timeout_wall:
            reason = 'MECHANICAL_SETTLING_WALL_WATCHDOG'
        if reason:
            return {'passed': False, 'reason': reason, **self.timing(sim, wall), **self.metrics}

        keys = ('selected', 'drive', 'body_velocity', 'odom_velocity', 'wheel_positions',
                'wheel_velocities', 'steering_positions', 'steering_targets', 'body_pose')
        valid = sample is not None and all(key in sample for key in keys)
        if valid:
            valid = (all(isinstance(sample[key], (tuple, list)) for key in keys)
                     and bool(sample.get('source_wall_times'))
                     and all(0 <= wall - stamp < self.data_age_wall
                             for stamp in sample['source_wall_times'])
                     and all(math.isfinite(v) for key in keys for v in sample[key])
                     and len(sample['wheel_positions']) == len(sample['wheel_velocities']) == 2
                     and len(sample['steering_positions']) == 2
                     and len(sample['steering_targets']) == 2
                     and len(sample['body_pose']) == 3
                     and len(sample['selected']) == len(sample['body_velocity']) == len(sample['odom_velocity']) == 3
                     and len(sample['drive']) == 2)
        quiet = (valid and all(abs(v) < 1e-6 for v in sample['selected'] + sample['drive'])
                 and all(abs(v) < self.body_tolerance
                         for v in sample['body_velocity'] + sample['odom_velocity']))
        if not quiet:
            self.samples.clear()
            return None
        if not self.samples or sim > self.samples[-1][0]:
            self.samples.append((sim, sample))
        # Keep one sample at/before the boundary, so a full physics window is proven.
        while len(self.samples) > 2 and self.samples[1][0] <= sim - self.window_sim:
            self.samples.popleft()
        span = sim - self.samples[0][0]
        if span < self.window_sim or len(self.samples) < 4:
            return None
        rows = [row[1] for row in self.samples]
        wheel_rates = [(max(row['wheel_positions'][i] for row in rows)
                        - min(row['wheel_positions'][i] for row in rows)) / span for i in (0, 1)]
        steer_spans = [max(row['steering_positions'][i] for row in rows)
                       - min(row['steering_positions'][i] for row in rows) for i in (0, 1)]
        target_spans = [max(row['steering_targets'][i] for row in rows)
                       - min(row['steering_targets'][i] for row in rows) for i in (0, 1)]
        xy_rate = math.hypot(*(max(row['body_pose'][i] for row in rows)
                              - min(row['body_pose'][i] for row in rows) for i in (0, 1))) / span
        # Locally unwrap yaw around the first sample (no +/-pi discontinuity).
        yaw0 = rows[0]['body_pose'][2]
        yaw = [math.atan2(math.sin(row['body_pose'][2] - yaw0),
                          math.cos(row['body_pose'][2] - yaw0)) for row in rows]
        yaw_rate = (max(yaw) - min(yaw)) / span
        self.metrics = {'wheel_position_drift_rates': wheel_rates,
                        'wheel_position_rate_limit': self.wheel_position_rate,
                        'wheel_rolling_drift_rates': (
                            [rate * self.wheel_radius for rate in wheel_rates]
                            if self.wheel_radius is not None else None),
                        'wheel_velocities': list(sample['wheel_velocities']),
                        'body_position_drift_rate': xy_rate, 'body_yaw_drift_rate': yaw_rate,
                        'body_velocity': list(sample['body_velocity']),
                        'odom_velocity': list(sample['odom_velocity']), 'stable_sim_window': span}
        if (max(wheel_rates) <= self.wheel_position_rate
                and max(steer_spans) <= self.steering_span
                and max(target_spans) <= self.steering_span
                and xy_rate <= self.body_position_rate
                and yaw_rate <= self.body_position_rate):
            return {'passed': True, 'reason': None, **self.timing(sim, wall), **self.metrics}
        return None
