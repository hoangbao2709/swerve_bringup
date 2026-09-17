#!/usr/bin/env python3
"""
Create a 2D occupancy map from the simulated 3D LiDAR.

Requirements:
- Gazebo + robot are already running.
- /lidar/points, /odom and TF odom -> base_footprint are available.
- ROS 2 Humble environment has been sourced.

This script starts the existing mapping pipeline:
    /lidar/points
        -> lidar_preprocessor
        -> /lidar/points_filtered
        -> pointcloud_to_laserscan
        -> /scan
        -> slam_toolbox
        -> /map

Commands while running:
    s  Save map to swerve_navigation/maps/warehouse
    q  Quit
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


WORKSPACE = Path.home() / "swerve_bringup"
DEFAULT_MAP_DIR = WORKSPACE / "swerve_navigation" / "maps"
DEFAULT_MAP_NAME = "warehouse"


RVIZ_CONFIG = r"""
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Plane: XY
      Reference Frame: <Fixed Frame>
      Value: true

    - Alpha: 0.85
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Value: true

    - Alpha: 1
      Class: rviz_default_plugins/LaserScan
      Color: 0; 255; 0
      Enabled: true
      Name: LaserScan
      Size (Pixels): 2
      Style: Points
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Value: true

    - Alpha: 0.25
      Class: rviz_default_plugins/PointCloud2
      Color Transformer: Intensity
      Enabled: true
      Name: PointCloud3D
      Position Transformer: XYZ
      Size (m): 0.02
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lidar/points
      Value: true

    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Value: true

    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Marker Scale: 0.5
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Value: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30

  Name: root

  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure

  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Angle: 0
      Scale: 30
      Target Frame: <Fixed Frame>

Window Geometry:
  Height: 900
  Width: 1400
"""


def run_capture(command):
    try:
        return subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=8,
            check=False,
        )
    except Exception as exc:
        print(f"[ERROR] {' '.join(command)}: {exc}")
        return None


def ros_topic_exists(topic):
    result = run_capture(["ros2", "topic", "list"])
    if not result:
        return False
    return topic in result.stdout.splitlines()


def ros_package_exists(package):
    result = run_capture(["ros2", "pkg", "prefix", package])
    return bool(result and result.returncode == 0)


def wait_for_topic(topic, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        if ros_topic_exists(topic):
            return True
        time.sleep(1)
    return False


def save_map(map_path):
    map_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"\n[SAVE] Saving map to: {map_path}")
    result = subprocess.run(
        [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            str(map_path),
        ],
        check=False,
    )

    yaml_path = map_path.with_suffix(".yaml")
    pgm_path = map_path.with_suffix(".pgm")

    if result.returncode == 0 and yaml_path.exists() and pgm_path.exists():
        print(f"[OK] {yaml_path}")
        print(f"[OK] {pgm_path}")
    else:
        print("[ERROR] Map save failed.")


def terminate_process(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass


def main():
    print("=" * 60)
    print(" SWERVE 3D LIDAR -> 2D MAP")
    print("=" * 60)

    if not WORKSPACE.exists():
        print(f"[ERROR] Workspace not found: {WORKSPACE}")
        return 1

    required_packages = [
        "swerve_bringup",
        "pointcloud_to_laserscan",
        "slam_toolbox",
        "nav2_map_server",
    ]

    missing = [p for p in required_packages if not ros_package_exists(p)]
    if missing:
        print("[ERROR] Missing ROS packages:")
        for package in missing:
            print(f"  - {package}")
        print("\nInstall missing packages, then source ROS and workspace again.")
        return 1

    print("[CHECK] Waiting for /lidar/points ...")
    if not wait_for_topic("/lidar/points", timeout=15):
        print("[ERROR] /lidar/points not found.")
        print("Start Gazebo + robot first.")
        return 1

    print("[OK] /lidar/points detected.")

    if not ros_topic_exists("/odom"):
        print("[WARN] /odom is not currently visible.")
        print("SLAM needs odometry + TF odom -> base_footprint.")

    rviz_file = Path("/tmp/swerve_mapping.rviz")
    rviz_file.write_text(RVIZ_CONFIG, encoding="utf-8")

    mapping_cmd = [
        "ros2",
        "launch",
        "swerve_bringup",
        "slam.launch.py",
        "use_sim_time:=true",
        "input_topic:=/lidar/points",
    ]

    print("\n[START] Mapping pipeline:")
    print(" ".join(mapping_cmd))

    mapping = subprocess.Popen(
        mapping_cmd,
        cwd=str(WORKSPACE),
        preexec_fn=os.setsid,
    )

    print("[CHECK] Waiting for /scan ...")
    if not wait_for_topic("/scan", timeout=30):
        print("[ERROR] /scan was not created.")
        terminate_process(mapping)
        return 1
    print("[OK] /scan detected.")

    print("[CHECK] Waiting for /map ...")
    if not wait_for_topic("/map", timeout=45):
        print("[ERROR] /map was not created.")
        print("Check TF odom -> base_footprint and /scan.")
        terminate_process(mapping)
        return 1
    print("[OK] /map detected.")

    rviz = subprocess.Popen(
        ["rviz2", "-d", str(rviz_file)],
        cwd=str(WORKSPACE),
        preexec_fn=os.setsid,
    )

    map_path = DEFAULT_MAP_DIR / DEFAULT_MAP_NAME

    print("\n" + "=" * 60)
    print("MAPPING IS RUNNING")
    print("Drive the robot slowly around the warehouse.")
    print("")
    print("  s + Enter : save map")
    print("  q + Enter : quit")
    print("")
    print(f"Default map path: {map_path}")
    print("=" * 60)

    try:
        while True:
            command = input("\nCommand [s/q]: ").strip().lower()

            if command == "s":
                save_map(map_path)
            elif command == "q":
                break
            elif command:
                print("Use 's' to save or 'q' to quit.")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        print("\n[STOP] Shutting down mapping...")
        terminate_process(rviz)
        terminate_process(mapping)

    return 0


if __name__ == "__main__":
    sys.exit(main())