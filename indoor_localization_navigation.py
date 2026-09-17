#!/usr/bin/env python3
import argparse
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener


def q_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class IndoorAGV(Node):
    def __init__(self):
        super().__init__("indoor_agv")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("v30e_tag_topic", "/v30e/tag_id")
        self.declare_parameter("v30e_pose_topic", "/v30e/pose")
        self.declare_parameter("global_odom_topic", "/odometry/v30e")
        self.declare_parameter("navigate_action", "/navigate_to_pose")

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_tag = None
        self.last_v30e_pose = None
        self.last_global_odom = None

        self.create_subscription(
            Int32,
            self.get_parameter("v30e_tag_topic").value,
            self._tag_cb,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("v30e_pose_topic").value,
            self._v30e_pose_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("global_odom_topic").value,
            self._odom_cb,
            20,
        )

        self.nav = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("navigate_action").value,
        )

        self.goal_handle = None
        self.goal_active = False
        self.distance_remaining = None

        self.get_logger().info(
            f"Localization: {self.map_frame}->{self.base_frame}; "
            "navigation: /navigate_to_pose"
        )

    def _tag_cb(self, msg):
        self.last_tag = msg.data

    def _v30e_pose_cb(self, msg):
        self.last_v30e_pose = msg

    def _odom_cb(self, msg):
        self.last_global_odom = msg

    def current_pose(self):
        """Authoritative pose: TF map -> base_footprint."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
            t = tf.transform.translation
            return t.x, t.y, q_to_yaw(tf.transform.rotation)
        except TransformException:
            if self.last_global_odom is None:
                return None
            p = self.last_global_odom.pose.pose
            return p.position.x, p.position.y, q_to_yaw(p.orientation)

    def print_pose(self):
        p = self.current_pose()
        if p is None:
            print("[LOC] Chưa có TF map->base_footprint hoặc /odometry/v30e")
            return
        print(
            f"[LOC] x={p[0]:.3f} m  y={p[1]:.3f} m  "
            f"yaw={math.degrees(p[2]):.2f} deg"
        )

    def print_v30e(self):
        print(f"[V30E] TagID={self.last_tag}")
        if self.last_v30e_pose is None:
            print("[V30E] Chưa có /v30e/pose")
            return
        p = self.last_v30e_pose.pose.pose
        c = self.last_v30e_pose.pose.covariance
        print(
            f"[V30E] x={p.position.x:.3f} y={p.position.y:.3f} "
            f"yaw={math.degrees(q_to_yaw(p.orientation)):.2f} deg "
            f"cov=({c[0]:.6f},{c[7]:.6f},{c[35]:.6f})"
        )

    def goto(self, x, y, yaw_deg):
        if self.goal_active:
            print("[NAV] Đang có goal active; cancel trước.")
            return

        if self.current_pose() is None:
            print("[NAV] Chưa có localization nên chưa gửi goal.")
            return

        print("[NAV] Chờ Nav2 /navigate_to_pose ...")
        if not self.nav.wait_for_server(timeout_sec=10.0):
            print("[NAV] Không tìm thấy Nav2 action server.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        yaw = math.radians(float(yaw_deg))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f"[NAV] Goal x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.2f} deg")
        f = self.nav.send_goal_async(goal, feedback_callback=self._feedback)
        f.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            print(f"[NAV] Send goal error: {e}")
            return

        if not handle.accepted:
            print("[NAV] Goal rejected")
            return

        self.goal_handle = handle
        self.goal_active = True
        print("[NAV] Goal accepted")
        f = handle.get_result_async()
        f.add_done_callback(self._result)

    def _feedback(self, msg):
        self.distance_remaining = float(msg.feedback.distance_remaining)
        print(
            f"\r[NAV] Remaining {self.distance_remaining:.2f} m",
            end="",
            flush=True,
        )

    def _result(self, future):
        print()
        try:
            status = future.result().status
        except Exception as e:
            print(f"[NAV] Result error: {e}")
            status = None

        names = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        print(f"[NAV] Result: {names.get(status, status)}")
        self.goal_handle = None
        self.goal_active = False
        self.distance_remaining = None

    def cancel(self):
        if self.goal_handle is None:
            print("[NAV] Không có goal active.")
            return
        self.goal_handle.cancel_goal_async()
        print("[NAV] Cancel requested")

    def status(self):
        self.print_pose()
        print(f"[V30E] last_tag={self.last_tag}")
        if self.goal_active:
            print(f"[NAV] ACTIVE remaining={self.distance_remaining}")
        else:
            print("[NAV] IDLE")


def interactive(node):
    print(
        "\nCommands:\n"
        "  pose\n"
        "  v30e\n"
        "  goto X Y YAW_DEG\n"
        "  cancel\n"
        "  status\n"
        "  quit\n"
    )

    while rclpy.ok():
        try:
            parts = input("agv> ").strip().split()
        except (KeyboardInterrupt, EOFError):
            break
        if not parts:
            continue

        cmd = parts[0].lower()
        try:
            if cmd == "pose":
                node.print_pose()
            elif cmd == "v30e":
                node.print_v30e()
            elif cmd == "goto" and len(parts) == 4:
                node.goto(float(parts[1]), float(parts[2]), float(parts[3]))
            elif cmd == "cancel":
                node.cancel()
            elif cmd == "status":
                node.status()
            elif cmd in ("q", "quit", "exit"):
                break
            else:
                print("Dùng: pose | v30e | goto X Y YAW_DEG | cancel | status | quit")
        except ValueError:
            print("X Y YAW phải là số.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"))
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else None)
    node = IndoorAGV()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()
    time.sleep(1.0)

    try:
        if args.goal:
            node.goto(*args.goal)
            while rclpy.ok():
                time.sleep(0.2)
        else:
            interactive(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
