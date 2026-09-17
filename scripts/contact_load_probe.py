#!/usr/bin/env python3
"""Diagnostic-only Gazebo contact load probe; no localization input."""
import csv, math, os, time
import rclpy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock

CASES={'stationary':(0.0,0.0,0.0),'X+':(0.2,0.0,0.0),'Y+':(0.0,0.2,0.0),'yaw+':(0.0,0.0,0.2)}
LINKS=('wheel_front_drive_link','wheel_rear_drive_link','wheel_front_left_link','wheel_front_right_link','wheel_rear_left_link','wheel_rear_right_link')

class Probe(Node):
    def __init__(self):
        super().__init__('contact_load_probe', parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
        if not self.get_parameter('use_sim_time').value: raise RuntimeError('use_sim_time false')
        self.data={link:{'count':0,'normal':0.0,'tangential':0.0} for link in LINKS}; self.clocks=[]
        qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock,'/clock',self.clock_cb,qos)
        for link in LINKS: self.create_subscription(ContactsState,f'/contact_load/{link}',lambda m,l=link:self.contact_cb(l,m),qos)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
    def clock_cb(self,m): self.clocks.append(m.clock.sec+m.clock.nanosec*1e-9); self.clocks=self.clocks[-20:]
    def contact_cb(self,link,msg):
        d=self.data[link]
        d['count']=len(msg.states); d['normal']=0.0; d['tangential']=0.0
        for s in msg.states:
            f=s.total_wrench.force; d['normal']+=abs(f.z); d['tangential']+=math.hypot(f.x,f.y)
    def spin_wall(self,s):
        end=time.monotonic()+s
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01)
    def run(self,case):
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and (len(self.clocks)<3 or self.clocks[-1]<=self.clocks[0]): self.spin_wall(.05)
        if len(self.clocks)<3 or self.clocks[-1]<=self.clocks[0]: raise RuntimeError('clock did not advance')
        start=self.get_clock().now().nanoseconds*1e-9; next_phase=start+1.0; end=next_phase+8.0; wall=time.monotonic(); cmd=CASES[case]; rows=[]
        while self.get_clock().now().nanoseconds*1e-9<end and time.monotonic()-wall<120:
            sim=self.get_clock().now().nanoseconds*1e-9; active=sim>=next_phase; m=Twist()
            if active:m.linear.x,m.linear.y,m.angular.z=cmd
            self.pub.publish(m); rclpy.spin_once(self,timeout_sec=.01)
            row={'case':case,'phase':'motion' if active else 'stationary','sim_time':sim,'cmd_x':m.linear.x,'cmd_y':m.linear.y,'cmd_wz':m.angular.z}
            for link,d in self.data.items():
                row[f'{link}_contact_count']=d['count']; row[f'{link}_normal_force_sum']=d['normal']; row[f'{link}_tangential_force_sum']=d['tangential']
                # Values represent the latest ContactsState sample. Do not
                # reset here: callback cadence is independent of probe rows.
            rows.append(row)
        self.pub.publish(Twist()); self.spin_wall(.5); return rows

def main():
    case=os.environ.get('ACCEPTANCE_CASE','X+')
    if case not in CASES: case='X+'
    out=os.environ.get('ACCEPTANCE_CASE_DIR','artifacts/contact_load'); os.makedirs(out,exist_ok=True)
    rclpy.init(); n=Probe()
    try: rows=n.run(case)
    finally:n.destroy_node();rclpy.shutdown()
    with open(os.path.join(out,'contact_load.csv'),'w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    return 0
if __name__=='__main__':raise SystemExit(main())
