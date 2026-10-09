from unittest.mock import patch

from django.core.management.base import CommandError
from django.test import TestCase

from twin.management.commands.sync_master_data import Command


class SyncMasterDataSafetyTests(TestCase):
    def test_essential_master_data_sync_failure_is_a_startup_failure(self):
        with patch('twin.warehouse_services.sync_from_layout', side_effect=RuntimeError('invalid canonical layout')):
            with self.assertRaisesRegex(CommandError, 'critical warehouse master-data synchronization failed'):
                Command().handle()
