#!/usr/bin/env python3
"""Repeatable accuracy benchmark for the simulated swerve stack.

Gazebo /get_entity_state is queried only by this evaluator.  It is never
published, injected into TF, or used as a Nav2 measurement.
"""
import argparse, csv, json, math, os, threading, time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Imu
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return (a + math.pi) % (2*math.pi) - math.pi
def xy(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
def stats(values):
    v = [float(x) for x in values if math.isfinite(float(x))]
    if not v: return {'mean': None, 'max': None, 'rmse': None, 'stddev': None}
    m = sum(v)/len(v)
    return {'mean': m, 'max': max(v), 'rmse': math.sqrt(sum(x*x for x in v)/len(v)),
            'stddev': math.sqrt(sum((x-m)**2 for x in v)/len(v))}


class Benchmark(Node):
    def __init__(self, model, timeout):
        super().__init__('swerve_accuracy_benchmark', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('FAIL: accuracy benchmark use_sim_time is false')
        self.model, self.timeout = model, timeout
        self.gt = self.odom = self.v30e = self.local_ekf = self.global_ekf = self.imu = None
        self.clock_samples = []
        self.plan_received = False
        self.last_cmd = None
        self.cmd_vel_seen_nonzero = False
        self.feedback_distance = None
        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(self, 'local_ekf', self.odom_msg(m)), 20)
        self.create_subscription(Odometry, '/odometry/v30e', lambda m: setattr(self, 'global_ekf', self.odom_msg(m)), 20)
        self.create_subscription(Imu, '/imu/data', lambda m: setattr(self, 'imu', (m.angular_velocity.z, yaw(m.orientation))), 20)
        self.create_subscription(Clock, '/clock', self.clock_cb, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/v30e/pose', self.v30e_cb, 20)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 20)
        self.state = self.create_client(GetEntityState, '/get_entity_state')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def sim_time(self):
        """Current ROS time in seconds (Gazebo /clock when use_sim_time=true)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def clock_cb(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_samples.append((value, time.monotonic()))
        self.clock_samples = self.clock_samples[-20:]

    def wait_for_sim_clock(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if len(self.clock_samples) >= 3 and self.clock_samples[-1][0] > self.clock_samples[0][0]:
                result = {'ros_clock_start': self.clock_samples[0][0],
                        'ros_clock_end': self.clock_samples[-1][0],
                        'sim_elapsed': self.clock_samples[-1][0] - self.clock_samples[0][0],
                        'wall_elapsed': self.clock_samples[-1][1] - self.clock_samples[0][1]}
                self.get_logger().info('use_sim_time=true first_clock=%.9f second_clock=%.9f clock_delta=%.9f wall_delta=%.9f measured_RTF=%.6f' %
                                       (result['ros_clock_start'], result['ros_clock_end'], result['sim_elapsed'], result['wall_elapsed'],
                                        result['sim_elapsed'] / result['wall_elapsed'] if result['wall_elapsed'] else 0.0))
                return result
            time.sleep(0.02)
        raise RuntimeError('FAIL: /clock did not advance while use_sim_time=true')

    @staticmethod
    def elapsed(start, end):
        return max(0.0, end - start)

    @staticmethod
    def odom_msg(m):
        p=m.pose.pose
        return (p.position.x,p.position.y,yaw(p.orientation),m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)
    def odom_cb(self, m): self.odom=self.odom_msg(m)[:3]
    def v30e_cb(self, m):
        p=m.pose.pose; self.v30e=(p.position.x,p.position.y,yaw(p.orientation))
    def plan_cb(self, _): self.plan_received = True
    def cmd_cb(self, m):
        self.last_cmd = (m.linear.x, m.linear.y, m.angular.z)
        if max(abs(v) for v in self.last_cmd) > 1e-3:
            self.cmd_vel_seen_nonzero = True
    def pose(self):
        try:
            t=self.tfbuf.lookup_transform('map','base_footprint',rclpy.time.Time()).transform
            return (t.translation.x,t.translation.y,yaw(t.rotation))
        except TransformException: return None
    def get_gt(self):
        if not self.state.wait_for_service(2.0): return None
        req=GetEntityState.Request(); req.name=self.model; req.reference_frame='world'
        f=self.state.call_async(req); deadline=time.monotonic()+2
        while not f.done() and time.monotonic()<deadline: time.sleep(.01)
        if not f.done() or not f.result().success: return None
        p=f.result().state.pose
        return (p.position.x,p.position.y,yaw(p.orientation))
    def stop(self):
        self.cmd_pub.publish(Twist()); time.sleep(.4); self.cmd_pub.publish(Twist())
    def nav_one(self, goal, trace):
        # Server discovery is not navigation progress, so retain a wall-clock
        # safety bound here.  Once accepted, all meaningful deadlines below
        # are based on ROS/Gazebo time.
        if not self.nav.wait_for_server(120): return {'status': 'NO_SERVER'}
        g=NavigateToPose.Goal(); g.pose=PoseStamped(); g.pose.header.frame_id='map'
        g.pose.header.stamp=self.get_clock().now().to_msg(); g.pose.pose.position.x=goal[0]; g.pose.pose.position.y=goal[1]
        g.pose.pose.orientation.z=math.sin(goal[2]/2); g.pose.pose.orientation.w=math.cos(goal[2]/2)
        def feedback_cb(msg):
            distance = getattr(msg.feedback, 'distance_remaining', None)
            if distance is not None:
                self.feedback_distance = float(distance)
        # Clear per-goal benchmark state before dispatch.  The action can be
        # accepted and publish feedback immediately on another executor
        # thread, so resetting after acceptance loses the first samples.
        self.plan_received = False
        self.feedback_distance = None
        self.cmd_vel_seen_nonzero = False
        f=self.nav.send_goal_async(g, feedback_callback=feedback_cb); wall_deadline=time.monotonic()+120
        while not f.done() and time.monotonic()<wall_deadline: time.sleep(.02)
        if not f.done() or f.result() is None: return {'status': 'REJECTED'}
        if not f.result().accepted: return {'status': 'REJECTED'}
        h=f.result(); rf=h.get_result_async()
        start_sim, start_wall = self.sim_time(), time.monotonic()
        sim_deadline = start_sim + self.timeout
        # This is deliberately much larger than the requested simulation
        # timeout.  It only protects against a stopped/broken simulator and
        # must never be the normal timeout criterion.
        wall_deadline = start_wall + max(1800.0, self.timeout * 60.0)
        action_state = 'ACTIVE'
        while not rf.done() and self.sim_time() < sim_deadline and time.monotonic() < wall_deadline:
            now = self.sim_time()
            gt = self.get_gt()
            loc = self.pose()
            odom = self.odom
            cmd = self.last_cmd or (0.0, 0.0, 0.0)
            trace.append({'sim_time': now, 'gt_x': gt[0] if gt else None,
                          'gt_y': gt[1] if gt else None, 'gt_yaw': gt[2] if gt else None,
                          'localization_x': loc[0] if loc else None,
                          'localization_y': loc[1] if loc else None,
                          'localization_yaw': loc[2] if loc else None,
                          'odom_x': odom[0] if odom else None,
                          'odom_y': odom[1] if odom else None,
                          'odom_yaw': odom[2] if odom else None,
                          'cmd_vel_x': cmd[0], 'cmd_vel_y': cmd[1], 'cmd_vel_yaw': cmd[2],
                          'local_ekf_x': self.local_ekf[0] if self.local_ekf else None,
                          'local_ekf_y': self.local_ekf[1] if self.local_ekf else None,
                          'local_ekf_yaw': self.local_ekf[2] if self.local_ekf else None,
                          'global_ekf_x': self.global_ekf[0] if self.global_ekf else None,
                          'global_ekf_y': self.global_ekf[1] if self.global_ekf else None,
                          'global_ekf_yaw': self.global_ekf[2] if self.global_ekf else None,
                          'imu_wz': self.imu[0] if self.imu else None,
                          'imu_yaw': self.imu[1] if self.imu else None,
                          'distance_remaining': self.feedback_distance,
                          'action_state': action_state})
            time.sleep(.10)
        end_sim, end_wall = self.sim_time(), time.monotonic()
        elapsed_sim = self.elapsed(start_sim, end_sim)
        elapsed_wall = self.elapsed(start_wall, end_wall)
        result = {'elapsed_sim_time': elapsed_sim, 'elapsed_wall_time': elapsed_wall,
                  'real_time_factor': elapsed_sim / elapsed_wall if elapsed_wall > 1e-9 else None,
                  'path_generated': self.plan_received,
                  'distance_remaining_last': self.feedback_distance}
        if not rf.done():
            action_state = 'TIMEOUT'
            h.cancel_goal_async()
            result['status'] = 'TIMEOUT'
            return result
        action_state = {GoalStatus.STATUS_SUCCEEDED:'SUCCEEDED',
                        GoalStatus.STATUS_ABORTED:'ABORTED',
                        GoalStatus.STATUS_CANCELED:'CANCELED'}.get(
                            rf.result().status, str(rf.result().status))
        result['status'] = {GoalStatus.STATUS_SUCCEEDED:'SUCCEEDED',
                            GoalStatus.STATUS_ABORTED:'ABORTED',
                            GoalStatus.STATUS_CANCELED:'CANCELED'}.get(
                            rf.result().status, str(rf.result().status))
        trace.append({'sim_time': end_sim, 'gt_x': None, 'gt_y': None, 'gt_yaw': None,
                      'localization_x': None, 'localization_y': None, 'localization_yaw': None,
                      'odom_x': None, 'odom_y': None, 'odom_yaw': None,
                      'cmd_vel_x': (self.last_cmd or (0, 0, 0))[0],
                      'cmd_vel_y': (self.last_cmd or (0, 0, 0))[1],
                      'cmd_vel_yaw': (self.last_cmd or (0, 0, 0))[2],
                      'distance_remaining': self.feedback_distance,
                      'action_state': action_state})
        return result

    def run(self, repeats, outdir):
        clock_validation = self.wait_for_sim_clock()
        start_sim, start_wall = self.sim_time(), time.monotonic()
        sim_deadline=start_sim+30.0
        wall_deadline=start_wall+300.0
        while (self.get_gt() is None or self.pose() is None or self.odom is None) and self.sim_time()<sim_deadline and time.monotonic()<wall_deadline:
            time.sleep(.2)
        if self.get_gt() is None or self.pose() is None or self.odom is None:
            raise RuntimeError('benchmark prerequisites unavailable: need GT, map->base_footprint, and /odom')
        # Goals stay in the open central area; each row is relative to the
        # measured current pose, so no world pose is hardcoded into localization.
        cases=[('X+',1,0,0),('X-',-1,0,0),('Y+',0,1,0),('Y-',0,-1,0),
               ('rotate',0,0,math.pi/2),('diagonal',1,1,0),('X+3',3,0,0),
               ('Y+3',0,3,0),('diagonal3',2.1,2.1,0),('yaw',0,0,math.pi)]
        rows=[]; traces=[]
        for i in range(repeats):
            name,dx,dy,da=cases[i%len(cases)]; start_gt=self.get_gt(); start_loc=self.pose(); start_odom=self.odom
            if not start_gt or not start_loc or not start_odom: continue
            # A world coordinate is not a map goal until alignment is proven.
            # Use the measured map pose as the goal origin and audit the
            # initial world/map transform separately.
            alignment_xy = xy(start_gt, start_loc)
            alignment_yaw = abs(wrap(start_gt[2] - start_loc[2]))
            goal=(start_loc[0]+dx,start_loc[1]+dy,wrap(start_loc[2]+da))
            trace=[]
            nav_result=self.nav_one(goal, trace); status=nav_result['status']; self.stop()
            end_gt=self.get_gt(); end_loc=self.pose(); end_odom=self.odom; end_v30e=self.v30e
            if not end_gt or not end_loc or not end_odom: continue
            # Odom is compared as displacement, not its arbitrary initial frame.
            odom_err=xy((end_gt[0]-start_gt[0],end_gt[1]-start_gt[1]),(end_odom[0]-start_odom[0],end_odom[1]-start_odom[1]))
            gt_yaw_displacement=abs(wrap(end_gt[2]-start_gt[2]))
            motion_ok=(xy(end_gt,start_gt) >= 0.05 if math.hypot(dx,dy) > 1e-9
                       else gt_yaw_displacement >= math.radians(5))
            gt_displacement_error = xy((end_gt[0]-start_gt[0], end_gt[1]-start_gt[1]), (dx, dy))
            gt_expected_yaw = wrap(start_gt[2] + da)
            gt_yaw_error = abs(wrap(end_gt[2] - gt_expected_yaw))
            row={'run':i+1,'test':name,'alignment_error_xy_initial':alignment_xy,
                 'alignment_error_yaw_initial':alignment_yaw,
                 'gt_goal_error':gt_displacement_error,
                 'localization_error_xy':xy(end_gt,end_loc),
                 'localization_error_yaw':abs(wrap(end_gt[2]-end_loc[2])),'odom_error_xy':odom_err,
                 'v30e_error_xy':xy(end_gt,end_v30e) if end_v30e else None,
                 'final_navigation_error_xy':gt_displacement_error,'final_yaw_error':gt_yaw_error,
                 'travelled_distance':xy(end_gt,start_gt),'action_status':status,
                 'gt_yaw_displacement':gt_yaw_displacement,'motion_ok':motion_ok,
                 'path_generated':nav_result.get('path_generated', False),
                 'cmd_vel_nonzero':self.cmd_vel_seen_nonzero,
                 'distance_remaining_last':nav_result.get('distance_remaining_last'),
                 'elapsed_sim_time':nav_result.get('elapsed_sim_time'),
                 'elapsed_wall_time':nav_result.get('elapsed_wall_time'),
                 'real_time_factor':nav_result.get('real_time_factor')}
            rows.append(row); traces.extend([dict(run=i+1, test=name, **s) for s in trace])
            print(json.dumps(row,sort_keys=True),flush=True)
        os.makedirs(outdir,exist_ok=True)
        with open(os.path.join(outdir,'accuracy_results.csv'),'w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=rows[0].keys() if rows else ['run']); w.writeheader(); w.writerows(rows)
        with open(os.path.join(outdir,'motion_trace.csv'),'w',newline='') as f:
            fields=['run','test','sim_time','gt_x','gt_y','gt_yaw','localization_x','localization_y',
                    'localization_yaw','odom_x','odom_y','odom_yaw','cmd_vel_x','cmd_vel_y',
                    'cmd_vel_yaw','local_ekf_x','local_ekf_y','local_ekf_yaw','global_ekf_x',
                    'global_ekf_y','global_ekf_yaw','imu_wz','imu_yaw','distance_remaining','action_state']
            w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(traces)
        if rows:
            r=rows[-1]; last=traces[-2] if len(traces)>1 else traces[-1]
            with open(os.path.join(outdir,'nav_xplus_diagnostic.csv'),'w',newline='') as f:
                fields=['run','test','gt_goal_error','localization_error_xy','final_yaw_error','action_status',
                        'gt_x','gt_y','gt_yaw','raw_odom_x','raw_odom_y','raw_odom_yaw',
                        'local_ekf_x','local_ekf_y','local_ekf_yaw','global_ekf_x','global_ekf_y',
                        'global_ekf_yaw','imu_wz','imu_yaw','v30e_x','v30e_y','v30e_yaw']
                w=csv.DictWriter(f,fieldnames=fields); w.writeheader()
                w.writerow({'run':r['run'],'test':r['test'],'gt_goal_error':r['gt_goal_error'],
                    'localization_error_xy':r['localization_error_xy'],'final_yaw_error':r['final_yaw_error'],
                    'action_status':r['action_status'],'gt_x':last.get('gt_x'),'gt_y':last.get('gt_y'),'gt_yaw':last.get('gt_yaw'),
                    'raw_odom_x':last.get('odom_x'),'raw_odom_y':last.get('odom_y'),'raw_odom_yaw':last.get('odom_yaw'),
                    'local_ekf_x':last.get('local_ekf_x'),'local_ekf_y':last.get('local_ekf_y'),'local_ekf_yaw':last.get('local_ekf_yaw'),
                    'global_ekf_x':last.get('global_ekf_x'),'global_ekf_y':last.get('global_ekf_y'),'global_ekf_yaw':last.get('global_ekf_yaw'),
                    'imu_wz':last.get('imu_wz'),'imu_yaw':last.get('imu_yaw'),
                    'v30e_x':end_v30e[0] if end_v30e else None,'v30e_y':end_v30e[1] if end_v30e else None,
                    'v30e_yaw':end_v30e[2] if end_v30e else None})
        metrics={k:stats([r[k] for r in rows if r[k] is not None]) for k in ('gt_goal_error','localization_error_xy','localization_error_yaw','odom_error_xy','final_navigation_error_xy','final_yaw_error','travelled_distance')}
        clock_validation['use_sim_time'] = bool(self.get_parameter('use_sim_time').value)
        clock_validation['clock_topic_seen'] = bool(self.clock_samples)
        clock_validation['rtf'] = clock_validation['sim_elapsed'] / clock_validation['wall_elapsed'] if clock_validation['wall_elapsed'] else None
        clock_validation['clock_verified'] = clock_validation['use_sim_time'] and clock_validation['clock_topic_seen'] and clock_validation['sim_elapsed'] > 0
        with open(os.path.join(outdir,'clock_validation.json'),'w') as f: json.dump(clock_validation,f,indent=2)
        summary={'runs':len(rows),'metrics':metrics,'clock_validation':clock_validation,'passed':len(rows)==repeats and all(
            r['action_status']=='SUCCEEDED' and r['motion_ok'] and r['final_navigation_error_xy']<=.05
            and r['localization_error_xy']<=.03 and r['final_yaw_error']<=.05
            for r in rows),'rows':rows}
        with open(os.path.join(outdir,'accuracy_summary.json'),'w') as f: json.dump(summary,f,indent=2)
        print(json.dumps(summary,indent=2),flush=True); return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument('--repeats',type=int,default=10); p.add_argument('--timeout',type=float,default=180); p.add_argument('--outdir',default='benchmark_results'); p.add_argument('--model',default='swerve_base'); a=p.parse_args()
    rclpy.init(); n=Benchmark(a.model,a.timeout); ex=MultiThreadedExecutor(4); ex.add_node(n); t=threading.Thread(target=ex.spin,daemon=True); t.start()
    try: n.run(a.repeats,a.outdir)
    finally: n.stop(); ex.shutdown(); n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
