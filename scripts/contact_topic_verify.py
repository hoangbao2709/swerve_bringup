#!/usr/bin/env python3
"""ROS-native equivalent of contact topic list/info/echo verification."""
import json, os, time
import rclpy
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock

LINKS=('wheel_front_drive_link','wheel_rear_drive_link','wheel_front_left_link','wheel_front_right_link','wheel_rear_left_link','wheel_rear_right_link')
class Verify(Node):
    def __init__(self):
        super().__init__('contact_topic_verify',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
        self.clock=[];self.messages={l:[] for l in LINKS}
        qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock,'/clock',lambda m:self.clock.append(m.clock.sec+m.clock.nanosec*1e-9),qos)
        for l in LINKS:self.create_subscription(ContactsState,f'/contact_load/{l}',lambda m,n=l:self.messages[n].append(m),qos)
    def run(self):
        end=time.monotonic()+20
        while time.monotonic()<end and len(self.clock)<3:self.get_logger().info('waiting /clock');rclpy.spin_once(self,timeout_sec=.05)
        start=self.get_clock().now().nanoseconds*1e-9; deadline=start+5
        while time.monotonic()<end+30 and self.get_clock().now().nanoseconds*1e-9<deadline:rclpy.spin_once(self,timeout_sec=.02)
        out={'use_sim_time':bool(self.get_parameter('use_sim_time').value),'topics':{},'clock_seen':len(self.clock)>=3}
        for l in LINKS:
            infos=self.get_publishers_info_by_topic(f'/contact_load/{l}')
            out['topics'][l]={'publisher_count':len(infos),'publishers':[{'node_name':i.node_name,'node_namespace':i.node_namespace,'topic_type':i.topic_type,'reliability':str(i.qos_profile.reliability),'durability':str(i.qos_profile.durability)} for i in infos],'messages_received':len(self.messages[l]),'nonempty_messages':sum(bool(m.states) for m in self.messages[l])}
        out['contact_telemetry_valid']=out['use_sim_time'] and out['clock_seen'] and all(v['publisher_count']>0 and v['nonempty_messages']>0 for v in out['topics'].values())
        return out
def main():
    out=os.environ.get('ACCEPTANCE_CASE_DIR','artifacts/contact_topic_verify');os.makedirs(out,exist_ok=True);rclpy.init();n=Verify()
    try:r=n.run()
    finally:n.destroy_node();rclpy.shutdown()
    with open(os.path.join(out,'contact_topic_verify.json'),'w') as f:json.dump(r,f,indent=2)
    print(json.dumps(r,indent=2));return 0 if r['contact_telemetry_valid'] else 1
if __name__=='__main__':raise SystemExit(main())
