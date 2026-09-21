from io import StringIO
from unittest.mock import patch

from django.contrib.auth.models import User
from django.core.management import call_command
from django.test import TestCase


class SeedDemoPasswordTests(TestCase):
    def test_seed_creates_once_and_preserves_an_operator_password(self):
        env = {
            'TWIN_ADMIN_USERNAME': 'seed-admin',
            'TWIN_ADMIN_EMAIL': 'seed-admin@example.com',
            'TWIN_ADMIN_PASSWORD': 'initial-secret-123',
        }
        first_output = StringIO()
        with patch.dict('os.environ', env, clear=False):
            call_command('seed_demo', stdout=first_output)

        user = User.objects.get(username='seed-admin')
        self.assertTrue(user.check_password('initial-secret-123'))
        self.assertNotIn('initial-secret-123', first_output.getvalue())

        user.set_password('operator-changed-secret-456')
        user.save(update_fields=['password'])

        second_output = StringIO()
        with patch.dict('os.environ', env, clear=False):
            call_command('seed_demo', stdout=second_output)

        user.refresh_from_db()
        self.assertTrue(user.check_password('operator-changed-secret-456'))
        self.assertFalse(user.check_password('initial-secret-123'))
        self.assertNotIn('operator-changed-secret-456', second_output.getvalue())
        self.assertNotIn('initial-secret-123', second_output.getvalue())
