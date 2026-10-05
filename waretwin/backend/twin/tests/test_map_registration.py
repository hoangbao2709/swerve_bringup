import math
from datetime import datetime, timezone
from unittest.mock import patch

from django.test import TestCase

from twin.map_registration import (
    GAZEBO_REGISTRATION_SOURCE,
    REGISTRATION_DRIFT_SAMPLES_REQUIRED,
    derive_active_from_canonical,
    transform_canonical_pose,
    update_gazebo_registration,
)
from twin.models import NavigationTag, RobotMapRegistration, Warehouse, WarehouseMap
from twin.navigation_targets import NavigationTargetError, navigation_tag_registry, resolve_navigation_target
from twin.runtime import runtime


class CanonicalActiveRegistrationTests(TestCase):
    def setUp(self):
        self.warehouse = Warehouse.objects.create(code='REG-TEST', name='Registration Test')
        self.layout = {'navigation_tags': []}
        self.warehouse_map = WarehouseMap.objects.create(
            warehouse=self.warehouse, layout=self.layout, draft=self.layout,
            revision=7, published_version=1, is_active=True,
        )
        self.tag = NavigationTag.objects.create(
            warehouse=self.warehouse, tag_id=22, family='APRILTAG',
            x=3.0, y=4.0, yaw=-0.2,
        )
        self.active_map = {
            'active_map_id': 'SLAM-session-42',
            'active_map_revision': 'session-session-42',
            'map_content_revision': 'content-a',
            'canonical_map_revision': 7,
            'map_source': 'SLAM_TOOLBOX',
            'map_sync_status': 'LOCAL_ONLY',
        }

    @staticmethod
    def pose_pair(canonical=(10.0, 5.0, 0.0), active=(1.0, 2.0, math.pi / 2)):
        timestamp = datetime.now(timezone.utc).isoformat()
        canonical_pose = {
            'x': canonical[0], 'y': canonical[1], 'yaw': canonical[2],
            'frame_id': 'map', 'map_id': 'CANONICAL', 'map_revision': '7',
            'map_source': 'CANONICAL', 'pose_source': 'GAZEBO_MODEL_STATES',
            'source_frame_id': 'world',
            'transform_source': 'VALIDATED_CANONICAL_WORLD_BUNDLE',
            'timestamp': timestamp, 'valid': True,
        }
        active_pose = {
            'x': active[0], 'y': active[1], 'yaw': active[2],
            'frame_id': 'map', 'map_id': 'SLAM-session-42',
            'map_revision': 'session-session-42', 'pose_source': 'TF',
            'timestamp': timestamp, 'valid': True,
        }
        return canonical_pose, active_pose

    def register(self, *, active_map=None, canonical_pose=None, active_pose=None, drift=None):
        canonical, active = self.pose_pair()
        return update_gazebo_registration(
            robot_id='R01', active_map=active_map or self.active_map,
            canonical_pose=canonical_pose or canonical, active_pose=active_pose or active,
            drift_state=drift if drift is not None else {},
        )

    def test_simulation_provider_derives_and_reuses_stable_se2_registration(self):
        transform = derive_active_from_canonical(
            {'x': 10, 'y': 5, 'yaw': 0}, {'x': 1, 'y': 2, 'yaw': math.pi / 2})
        self.assertAlmostEqual(transform['tx'], 6.0)
        self.assertAlmostEqual(transform['ty'], -8.0)
        self.assertAlmostEqual(transform['yaw'], math.pi / 2)
        same_pose = transform_canonical_pose({'x': 10, 'y': 5, 'yaw': 0}, transform)
        self.assertAlmostEqual(same_pose['x'], 1.0)
        self.assertAlmostEqual(same_pose['y'], 2.0)
        self.assertAlmostEqual(same_pose['yaw'], math.pi / 2)

        first = self.register()
        self.assertTrue(first['changed'])
        self.assertTrue(first['created'])
        registration = first['registration']
        self.assertEqual(registration.source, GAZEBO_REGISTRATION_SOURCE)
        self.assertEqual(registration.registration_revision, 1)
        self.assertAlmostEqual(registration.tx, 6.0)
        self.assertAlmostEqual(registration.ty, -8.0)

        # Content churn does not participate in registration identity.
        second = self.register(active_map={**self.active_map, 'map_content_revision': 'content-b'})
        self.assertFalse(second['changed'])
        self.assertEqual(second['registration'].pk, registration.pk)
        self.assertEqual(second['registration'].registration_revision, 1)

    def test_canonical_point_and_tag_use_the_same_current_registration(self):
        self.register()
        point = resolve_navigation_target(
            robot_id='R01', source_type='CANONICAL_MAP_POINT', active_map=self.active_map,
            x=10, y=5, yaw=0, source_map_id='CANONICAL', source_map_revision='7')
        tag = resolve_navigation_target(
            robot_id='R01', source_type='TAG', active_map=self.active_map, tag_id=22)

        self.assertAlmostEqual(point['x'], 1.0)
        self.assertAlmostEqual(point['y'], 2.0)
        self.assertAlmostEqual(point['yaw'], math.pi / 2)
        self.assertAlmostEqual(tag['x'], 2.0)
        self.assertAlmostEqual(tag['y'], -5.0)
        self.assertAlmostEqual(tag['yaw'], -0.2 + math.pi / 2)
        self.assertEqual(point['metadata']['registration_revision'], tag['metadata']['registration_revision'])
        self.assertEqual(point['metadata']['registration_source'], GAZEBO_REGISTRATION_SOURCE)
        self.assertEqual(tag['metadata']['registration_source'], GAZEBO_REGISTRATION_SOURCE)

    def test_active_slam_point_does_not_require_canonical_registration(self):
        target = resolve_navigation_target(
            robot_id='R01', source_type='ACTIVE_MAP_POINT', active_map=self.active_map,
            source_map_id='SLAM-session-42', source_map_revision='session-session-42',
            x=0.8, y=1.2, yaw=-0.1)
        self.assertEqual((target['x'], target['y'], target['yaw']), (0.8, 1.2, -0.1))
        self.assertEqual(target['map_id'], self.active_map['active_map_id'])

    def test_material_registration_drift_updates_only_after_hysteresis(self):
        self.register()
        drift = {}
        for sample in range(REGISTRATION_DRIFT_SAMPLES_REQUIRED - 1):
            canonical, active = self.pose_pair(active=(1.25, 2.0, math.pi / 2))
            result = self.register(canonical_pose=canonical, active_pose=active, drift=drift)
            self.assertFalse(result['changed'], sample)
            self.assertEqual(result['registration'].registration_revision, 1)
        canonical, active = self.pose_pair(active=(1.25, 2.0, math.pi / 2))
        updated = self.register(canonical_pose=canonical, active_pose=active, drift=drift)
        self.assertTrue(updated['changed'])
        self.assertEqual(updated['registration'].registration_revision, 2)
        self.assertAlmostEqual(updated['registration'].tx, 6.25)

    def test_new_slam_session_cannot_reuse_old_registration(self):
        self.register()
        new_map = {**self.active_map, 'active_map_id': 'SLAM-session-43',
                   'active_map_revision': 'session-session-43'}
        with self.assertRaises(NavigationTargetError) as point_error:
            resolve_navigation_target(
                robot_id='R01', source_type='CANONICAL_MAP_POINT', active_map=new_map,
                x=10, y=5, yaw=0, source_map_id='CANONICAL', source_map_revision='7')
        self.assertEqual(point_error.exception.code, 'MAP_REGISTRATION_REQUIRED')
        with self.assertRaises(NavigationTargetError) as tag_error:
            resolve_navigation_target(robot_id='R01', source_type='TAG', active_map=new_map, tag_id=22)
        self.assertEqual(tag_error.exception.code, 'TAG_MAP_REGISTRATION_REQUIRED')
        self.assertEqual(RobotMapRegistration.objects.filter(is_active=True).count(), 1)
        self.assertFalse(navigation_tag_registry(new_map, robot_id='R01')['compatible'])

    def test_provider_rejects_stale_or_wrong_source_pose_pair(self):
        canonical, active = self.pose_pair()
        canonical['timestamp'] = '2020-01-01T00:00:00+00:00'
        self.assertIsNone(self.register(canonical_pose=canonical, active_pose=active))
        canonical, active = self.pose_pair()
        canonical['transform_source'] = 'UNVALIDATED'
        self.assertIsNone(self.register(canonical_pose=canonical, active_pose=active))

    def test_registration_revision_change_invalidates_only_canonical_derived_previews(self):
        previews = {
            ('R01', 'active'): {'source_type': 'ACTIVE_MAP_POINT'},
            ('R01', 'canonical'): {'source_type': 'CANONICAL_MAP_POINT'},
            ('R01', 'tag'): {'source_type': 'TAG'},
        }
        results = dict(previews)
        approved = dict(previews)
        invalidations = {}
        with patch.object(runtime, 'path_preview_requests', dict(previews)), \
                patch.object(runtime, 'path_preview_results', results), \
                patch.object(runtime, 'approved_path_previews', approved), \
                patch.object(runtime, 'path_preview_invalidations', invalidations):
            runtime.invalidate_registration_path_previews('R01', 'registration revision changed')

        self.assertIn(('R01', 'active'), results)
        self.assertIn(('R01', 'active'), approved)
        self.assertNotIn(('R01', 'canonical'), results)
        self.assertNotIn(('R01', 'tag'), results)
        self.assertIn(('R01', 'canonical'), invalidations)
        self.assertIn(('R01', 'tag'), invalidations)
