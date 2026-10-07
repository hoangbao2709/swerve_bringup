#!/usr/bin/env python3
"""Apply a reversible, visualization-only Gazebo Classic demo camera pose."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path


def camera_pose(x: float, y: float, z: float, yaw: float) -> tuple[float, ...]:
    """Return a fixed elevated camera pose looking toward the robot area."""
    target_x = x + 1.5 * math.cos(yaw)
    target_y = y + 1.5 * math.sin(yaw)
    target_z = z
    horizontal_offset = 11.0
    camera_z = target_z + 15.0
    camera_x = target_x - horizontal_offset * math.cos(yaw + math.pi / 4)
    camera_y = target_y - horizontal_offset * math.sin(yaw + math.pi / 4)
    dx, dy, dz = target_x - camera_x, target_y - camera_y, target_z - camera_z
    camera_yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    half_yaw, half_pitch = camera_yaw / 2.0, pitch / 2.0
    qx = -math.sin(half_yaw) * math.sin(half_pitch)
    qy = math.cos(half_yaw) * math.sin(half_pitch)
    qz = math.sin(half_yaw) * math.cos(half_pitch)
    qw = math.cos(half_yaw) * math.cos(half_pitch)
    return camera_x, camera_y, camera_z, qx, qy, qz, qw


def spawn_pose(world_path: Path, robot_id: str) -> tuple[float, float, float, float]:
    manifest_path = world_path.parent.parent / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"verified artifact manifest not found: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for robot in manifest.get("robots", []):
        if robot.get("id") == robot_id:
            pose = robot.get("pose")
            if isinstance(pose, list) and len(pose) >= 4:
                return tuple(float(value) for value in pose[:4])  # type: ignore[return-value]
    raise ValueError(f"robot {robot_id!r} has no pose in {manifest_path}")


def gazebotopics(world_name: str) -> tuple[str, str]:
    listed = subprocess.run(
        ["gz", "topic", "-l", "-w", world_name], check=True, text=True,
        capture_output=True, timeout=8,
    ).stdout.splitlines()
    topic = next((line.strip() for line in listed if line.strip().endswith("/user_camera/joy_pose")), None)
    if not topic:
        raise RuntimeError(f"Gazebo user camera pose topic not present for world {world_name}")
    info = subprocess.run(["gz", "topic", "-i", topic], check=True, text=True,
                          capture_output=True, timeout=8).stdout
    if "gazebo.msgs.Pose" not in info:
        raise RuntimeError(f"unexpected Gazebo camera topic type: {info.strip()}")
    subscribers = info.partition("Subscribers:")[2].strip()
    if not subscribers:
        raise RuntimeError("Gazebo GUI camera is not subscribed to the pose topic")
    return topic, info


def apply_camera(world_path: Path, robot_id: str, timeout_s: float) -> str:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None or not world.get("name"):
        raise ValueError(f"world name not found in {world_path}")
    pose = spawn_pose(world_path, robot_id)
    topic = ""
    deadline = time.monotonic() + timeout_s
    last_error = "camera topic not available"
    while time.monotonic() < deadline:
        try:
            topic, _ = gazebotopics(world.get("name", ""))
            break
        except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
            last_error = str(exc)
            time.sleep(1)
    if not topic:
        raise RuntimeError(last_error)

    camera = camera_pose(pose[0], pose[1], pose[2], pose[3])
    x, y, z, qx, qy, qz, qw = camera
    message = (
        "position {\n"
        f"  x: {x:.6f}\n  y: {y:.6f}\n  z: {z:.6f}\n"
        "}\norientation {\n"
        f"  x: {qx:.9f}\n  y: {qy:.9f}\n  z: {qz:.9f}\n  w: {qw:.9f}\n"
        "}\n"
    )
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", prefix="waretwin-demo-camera-", suffix=".pbtxt") as payload:
        payload.write(message)
        payload.flush()
        result = subprocess.run(
            ["gz", "topic", "-p", topic, "-f", payload.name], check=True,
            text=True, capture_output=True, timeout=10,
        )
    return f"{topic} ({x:.1f}, {y:.1f}, {z:.1f}) status={result.returncode}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--robot-id", default="R01")
    parser.add_argument("--timeout", default=45.0, type=float)
    args = parser.parse_args()
    try:
        print(apply_camera(args.world.resolve(), args.robot_id, args.timeout))
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, ET.ParseError) as exc:
        print(f"DEMO_CAMERA=FAIL {exc}")
        return 1
    print("DEMO_CAMERA=PASS visualization-only Gazebo camera pose applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
