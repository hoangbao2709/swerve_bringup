#!/usr/bin/env python3
"""Audit generated SDF wheel-cylinder contact heights at zero joint angles."""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


TARGETS = (
    "wheel_front_drive_link", "wheel_rear_drive_link",
    "wheel_front_left_link", "wheel_front_right_link",
    "wheel_rear_left_link", "wheel_rear_right_link",
)


def pose(node):
    values = [float(v) for v in (node.findtext("pose") or "0 0 0 0 0 0").split()]
    return values


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def matvec(matrix, vector):
    return tuple(sum(row[i] * vector[i] for i in range(3)) for row in matrix)


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def compose(parent, child):
    pt, pr = parent
    ct = child[0:3]
    roll, pitch, yaw = child[3:]
    child_rotation = rotation(roll, pitch, yaw)
    translated = matvec(pr, ct)
    return ((pt[0] + translated[0], pt[1] + translated[1], pt[2] + translated[2]), matmul(pr, child_rotation))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sdf")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    model = ET.parse(args.sdf).getroot().find("model")
    links = {link.get("name"): link for link in model.findall("link")}
    joints = {joint.findtext("child"): joint for joint in model.findall("joint")}
    identity = ((0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))

    def link_frame(name):
        joint = joints.get(name)
        if joint is None:
            return identity
        return compose(link_frame(joint.findtext("parent")), pose(joint))

    records = []
    for name in TARGETS:
        link = links[name]
        collision = next(c for c in link.findall("collision") if c.find("geometry/cylinder") is not None)
        cylinder = collision.find("geometry/cylinder")
        frame = compose(link_frame(name), pose(collision))
        radius, length = float(cylinder.findtext("radius")), float(cylinder.findtext("length"))
        # A cylinder's local axis is Z.  Its vertical half-extent follows the
        # projection of axial and radial components onto world Z.
        axis_z = frame[1][2][2]
        vertical_extent = abs(axis_z) * length / 2 + math.sqrt(1 - axis_z * axis_z) * radius
        record = {
            "link": name, "collision": collision.get("name"),
            "collision_origin_base_footprint_m": list(frame[0]),
            "collision_orientation_rpy": pose(collision)[3:],
            "radius_m": radius, "length_m": length,
            "calculated_lowest_z_base_footprint_m": frame[0][2] - vertical_extent,
            # base_link is 0.245 m above base_footprint in this generated model.
            "calculated_lowest_z_base_link_m": frame[0][2] - vertical_extent - 0.245,
        }
        if "drive" not in name:
            prefix = name.removesuffix("_link")
            fork = f"{prefix}_fork_link"
            axle_frame, swivel_frame = link_frame(name), link_frame(fork)
            roll_joint = joints[name]
            offset = pose(roll_joint)[0:2]
            record.update({
                "swivel_axis_position_base_footprint_m": list(swivel_frame[0]),
                "wheel_axle_position_base_footprint_m": list(axle_frame[0]),
                "horizontal_offset_vector_local_m": offset,
                "trail_m": math.hypot(*offset),
            })
        records.append(record)
    drives = [r["calculated_lowest_z_base_link_m"] for r in records if "drive" in r["link"]]
    casters = [r["calculated_lowest_z_base_link_m"] for r in records if "drive" not in r["link"]]
    drive_reference = sum(drives) / len(drives)
    max_delta_m = max(abs(z - drive_reference) for z in casters)
    trails = [r["trail_m"] for r in records if "trail_m" in r]
    result = {"wheels": records, "drive_contact_z_base_link_m": drive_reference,
              "max_caster_delta_mm": max_delta_m * 1000,
              "acceptance_max_delta_mm": 0.5, "current_model_trail_m": max(trails) if trails else None,
              "pass": max_delta_m <= 0.0005}
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
    for record in records:
        print(f"{record['link']}: collision={record['collision']} origin={record['collision_origin_base_footprint_m']} "
              f"rpy={record['collision_orientation_rpy']} radius={record['radius_m']} length={record['length_m']} "
              f"lowest_z_base_link={record['calculated_lowest_z_base_link_m']:.6f}")
        if "trail_m" in record:
            print(f"  swivel_axis={record['swivel_axis_position_base_footprint_m']} axle={record['wheel_axle_position_base_footprint_m']} "
                  f"offset_local_xy={record['horizontal_offset_vector_local_m']} trail={record['trail_m']:.6f} m")
    print(f"max caster-drive delta: {result['max_caster_delta_mm']:.3f} mm; pass={result['pass']}")
    raise SystemExit(0 if result["pass"] else 2)


if __name__ == "__main__":
    main()
