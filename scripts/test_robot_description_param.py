#!/usr/bin/env python3
"""Exercise ROS 2's command-line parameter parser with a generated URDF.

Run one input per process: rclpy must parse the same ``-p name:=value`` form
that gazebo_ros2_control uses internally before a node can be created.
"""
import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    args = parser.parse_args()
    xml = args.urdf.read_text(encoding="utf-8")
    node = None
    try:
        rclpy.init(args=["robot_description_param_probe", "--ros-args", "-p",
                         f"robot_description:={xml}"])
        node = Node("robot_description_param_probe")
        value = node.declare_parameter("robot_description", "").value
        if not isinstance(value, str) or not value:
            raise RuntimeError("robot_description was not a non-empty string")
        ET.fromstring(value)
        print(f"PARAM_PARSE=PASS file={args.urdf} input_bytes={len(xml)} parsed_bytes={len(value)}")
    except Exception as exc:
        print(f"PARAM_PARSE=FAIL file={args.urdf} error={type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
