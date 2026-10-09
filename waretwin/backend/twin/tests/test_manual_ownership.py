from django.test import SimpleTestCase

from twin.manual_ownership import acquire, active_owner, refresh, release


class ManualOwnershipLeaseTests(SimpleTestCase):
    def test_control_must_be_acquired_and_expires_deterministically(self):
        owners = {}
        accepted, error = refresh(owners, robot_id='R01', channel_name='client-a',
                                  lease_id='lease-a', now=10.0)
        self.assertFalse(accepted)
        self.assertEqual(error, 'MANUAL_LEASE_EXPIRED')

        accepted, error = acquire(owners, robot_id='R01', channel_name='client-a',
                                  lease_id='lease-a', identity='operator-a', now=10.0, ttl=0.5)
        self.assertTrue(accepted)
        self.assertIsNone(error)
        self.assertEqual(active_owner(owners, 'R01', 10.49)['identity'], 'operator-a')
        self.assertIsNone(active_owner(owners, 'R01', 10.5))
        self.assertNotIn('R01', owners)

    def test_second_operator_cannot_override_owner_but_can_acquire_after_release(self):
        owners = {}
        acquire(owners, robot_id='R01', channel_name='client-a', lease_id='lease-a',
                identity='operator-a', now=1.0)
        accepted, error = acquire(owners, robot_id='R01', channel_name='client-b',
                                  lease_id='lease-b', identity='operator-b', now=1.1)
        self.assertFalse(accepted)
        self.assertEqual(error, 'MANUAL_CONTROL_OWNED')
        self.assertEqual(owners['R01']['channel_name'], 'client-a')

        self.assertEqual(release(owners, robot_id='R01', channel_name='client-a',
                                 lease_id='lease-a'), (True, None))
        accepted, error = acquire(owners, robot_id='R01', channel_name='client-b',
                                  lease_id='lease-b', identity='operator-b', now=1.2)
        self.assertTrue(accepted)
        self.assertIsNone(error)

    def test_stale_or_foreign_lease_cannot_refresh_or_release_an_owner(self):
        owners = {}
        acquire(owners, robot_id='R01', channel_name='client-a', lease_id='lease-a',
                identity='operator-a', now=1.0)
        self.assertEqual(refresh(owners, robot_id='R01', channel_name='client-b',
                                 lease_id='lease-a', now=1.1),
                         (False, 'MANUAL_CONTROL_NOT_OWNER'))
        self.assertEqual(release(owners, robot_id='R01', channel_name='client-b',
                                 lease_id='lease-a'),
                         (False, 'MANUAL_CONTROL_NOT_OWNER'))
        self.assertEqual(owners['R01']['channel_name'], 'client-a')

    def test_stop_can_safely_invalidate_an_owner_without_granting_control(self):
        owners = {}
        acquire(owners, robot_id='R01', channel_name='client-a', lease_id='lease-a',
                identity='operator-a', now=1.0)
        self.assertEqual(release(owners, robot_id='R01', channel_name='client-b', require_owner=False),
                         (True, None))
        self.assertIsNone(active_owner(owners, 'R01', 1.1))
