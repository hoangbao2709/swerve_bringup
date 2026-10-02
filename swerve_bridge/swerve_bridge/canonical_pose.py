"""Validated Gazebo world -> canonical identity (no SLAM-origin assumptions)."""
import hashlib
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

from .coordinates import quaternion_yaw


class GazeboCanonicalAlignment:
    def __init__(self, world_file, robot_id, model_name):
        world = Path(world_file).resolve()
        root = world.parent.parent
        manifest = json.loads((root / 'manifest.json').read_text())
        self.revision = str(manifest['revision'])
        self.frame_id = manifest['frame_id']
        if self.frame_id != 'map' or manifest.get('units') != 'm':
            raise ValueError('canonical bundle must declare map/metres')
        roles = manifest['artifacts']
        if world != (root / roles['gazebo_world']).resolve():
            raise ValueError('running world is not the canonical bundle world')
        for role in ('gazebo_world', 'canonical_map', 'gazebo_manifest'):
            relative = roles[role]
            artifact = (root / relative).resolve()
            expected = manifest.get('sha256', {}).get(relative)
            if not artifact.is_relative_to(root) or not expected:
                raise ValueError('missing canonical artifact hash')
            if hashlib.sha256(artifact.read_bytes()).hexdigest() != expected:
                raise ValueError(f'{role} hash mismatch')
        canonical = json.loads((root / roles['canonical_map']).read_text())
        gazebo = json.loads((root / roles['gazebo_manifest']).read_text())
        if str(canonical['revision']) != self.revision or canonical['frame_id'] != self.frame_id:
            raise ValueError('canonical revision/frame mismatch')
        floors = {str(f['id']): f for f in canonical['floors']}
        world_floors = {str(f['id']): f for f in gazebo['floors']}
        if floors.keys() != world_floors.keys() or any(
                f['boundary'] != world_floors[k]['boundary']
                or f.get('holes', []) != world_floors[k].get('holes', [])
                for k, f in floors.items()):
            raise ValueError('canonical/world floor geometry mismatch')
        bounds = manifest['gazebo_bounds']
        origin = canonical['origin']
        expected_bounds = dict(min_x=origin['x'], min_y=origin['y'],
                               max_x=origin['x'] + canonical['width'],
                               max_y=origin['y'] + canonical['height'])
        if any(abs(float(bounds[k]) - float(v)) > 1e-6 for k, v in expected_bounds.items()):
            raise ValueError('canonical/world bounds mismatch')
        if not any(str(r['id']) == robot_id for r in manifest['robots']):
            raise ValueError('robot is not registered in this bundle')
        self.model_name = model_name
        self.world_file = str(world)
        self.anchors = {}
        for model in ET.parse(world).getroot().findall('.//world/model'):
            if model.findtext('static') == 'true' and model.find('pose') is not None:
                values = [float(v) for v in model.findtext('pose').split()]
                self.anchors[model.attrib['name']] = (values[0], values[1], values[5])
        if len(self.anchors) < 2:
            raise ValueError('world needs at least two static alignment anchors')
        # Verify the identity against canonical geometry itself, rather than
        # assuming that matching frame labels or a spawn pose establish it.
        for floor_id, floor in floors.items():
            ring = floor['boundary']
            safe_id = re.sub(r'[^A-Za-z0-9_.-]+', '_', floor_id)
            for index, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
                ax, ay = (a['x'], a['y']) if isinstance(a, dict) else a
                bx, by = (b['x'], b['y']) if isinstance(b, dict) else b
                expected = ((ax + bx) / 2, (ay + by) / 2, math.atan2(by - ay, bx - ax))
                actual = self.anchors.get(f'floor_{safe_id}_boundary_wall_{index}')
                if actual is None or any(abs(x-y) > 1e-6 for x, y in zip(actual, expected)):
                    raise ValueError('Gazebo boundary anchors do not establish canonical/world identity')

    def pose(self, states, timestamp):
        poses = dict(zip(states.name, states.pose))
        matched = []
        for name, expected in self.anchors.items():
            actual = poses.get(name)
            if actual is None:
                continue
            yaw_error = math.atan2(math.sin(quaternion_yaw(actual.orientation) - expected[2]),
                                   math.cos(quaternion_yaw(actual.orientation) - expected[2]))
            if math.hypot(actual.position.x - expected[0], actual.position.y - expected[1]) > 1e-4 or abs(yaw_error) > 1e-4:
                return None
            matched.append(expected)
        if len(matched) < 2 or not any(math.hypot(p[0] - matched[0][0], p[1] - matched[0][1]) > 1 for p in matched[1:]):
            return None
        robot = poses.get(self.model_name)
        if robot is None:
            return None
        values = (robot.position.x, robot.position.y, robot.position.z, quaternion_yaw(robot.orientation))
        if not all(math.isfinite(v) for v in values):
            return None
        return dict(x=values[0], y=values[1], z=values[2], yaw=values[3],
                    frame_id=self.frame_id, map_id='CANONICAL', map_revision=self.revision,
                    map_source='CANONICAL', pose_source='GAZEBO_MODEL_STATES',
                    source_frame_id='world', transform_source='VALIDATED_CANONICAL_WORLD_BUNDLE',
                    timestamp=timestamp, valid=True)
