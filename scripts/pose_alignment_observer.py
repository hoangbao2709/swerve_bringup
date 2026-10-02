#!/usr/bin/env python3
"""Independent live Gazebo/TF truth for browser pose alignment acceptance."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--model', default='swerve_base')
    parser.add_argument('--allow-placement', action='store_true',
                        help='allow fresh acceptance fixture requests to move the Gazebo entity')
    args = parser.parse_args()
    output = Path(args.output)
    rclpy.init()
    node = rclpy.create_node('web_pose_alignment_observer')
    tf = Buffer()
    listener = TransformListener(tf, node)
    truth = {'placement_enabled': args.allow_placement}
    current_pose = None
    placement_id = None
    placement_client = node.create_client(SetEntityState, '/set_entity_state')

    def models(msg):
        nonlocal current_pose
        if args.model not in msg.name:
            return
        index = msg.name.index(args.model)
        p, v = msg.pose[index], msg.twist[index]
        current_pose = p
        truth.update(world=dict(x=p.position.x, y=p.position.y, yaw=yaw(p.orientation)),
                     world_received_at_ms=time.time()*1000,
                     stopped=math.hypot(v.linear.x, v.linear.y) <= .005 and abs(v.angular.z) <= .005)

    node.create_subscription(ModelStates, '/model_states', models, qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', lambda m: truth.update(sim_time=m.clock.sec+m.clock.nanosec/1e9), qos_profile_sensor_data)
    node.create_subscription(String, '/command_owner', lambda m: truth.update(owner=m.data), 10)
    node.create_subscription(Twist, '/cmd_vel_selected', lambda m: truth.update(
        selected_zero=max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z)) < 1e-8), 10)
    last_write = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            request_file = output.parent / 'placement-request.json'
            if args.allow_placement and request_file.exists():
                try:
                    placement = json.loads(request_file.read_text())
                except json.JSONDecodeError:
                    placement = None
                if placement and placement['id'] != placement_id and current_pose is not None:
                    age_ms = time.time()*1000 - float(placement.get('requested_at_ms', 0))
                    if not 0 <= age_ms <= 30000:
                        placement_id = placement['id']
                        continue
                    if not truth.get('selected_zero') or not truth.get('stopped'):
                        raise RuntimeError('refusing entity placement while robot is moving or commanded')
                    if not placement_client.service_is_ready():
                        raise RuntimeError('Gazebo entity placement service is unavailable')
                    placement_id = placement['id']
                    req = SetEntityState.Request()
                    req.state.name = args.model
                    req.state.reference_frame = 'world'
                    req.state.pose.position.x = float(placement['x'])
                    req.state.pose.position.y = float(placement['y'])
                    req.state.pose.position.z = current_pose.position.z
                    req.state.pose.orientation.z = math.sin(float(placement['yaw']) / 2)
                    req.state.pose.orientation.w = math.cos(float(placement['yaw']) / 2)
                    def placed(future, request_id=placement_id):
                        truth['placement'] = dict(id=request_id, success=future.result().success)
                    placement_client.call_async(req).add_done_callback(placed)
            if time.monotonic() - last_write < .1:
                continue
            last_write = time.monotonic()
            try:
                transform = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                t = transform.transform
                truth['slam'] = dict(x=t.translation.x, y=t.translation.y, yaw=yaw(t.rotation))
                truth['slam_stamp'] = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
            except Exception:
                truth.pop('slam', None)
            truth['observer_at_ms'] = time.time()*1000
            temp = output.with_suffix('.tmp')
            temp.write_text(json.dumps(truth))
            temp.replace(output)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
