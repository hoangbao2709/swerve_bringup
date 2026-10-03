from unittest.mock import AsyncMock, patch

from django.contrib.auth.models import User
from django.test import Client, TestCase

from accounts.models import ApiToken
from twin.runtime import runtime


class ClearEmergencyStopAcknowledgementTests(TestCase):
    def setUp(self):
        user = User.objects.create_user(username='clear-estop-test', password='test-only-password')
        token = ApiToken.issue(user)
        self.client = Client(HTTP_AUTHORIZATION=f'Bearer {token.key}')

    def post_clear(self):
        return self.client.post('/api/robots/R01/clear-emergency-stop')

    def test_clear_succeeds_only_with_authoritative_applied_result(self):
        gateway = type('Gateway', (), {'request_control': AsyncMock(return_value={
            'ok': True, 'result': {'code': 'CLEAR_ESTOP_APPLIED',
                'emergency_stop_active': False, 'pre_stop_navigation_terminal': True,
                'message': 'latch cleared'},
        })})()
        with patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime.engine, 'emit') as emit:
            response = self.post_clear()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()['code'], 'CLEAR_ESTOP_APPLIED')
        self.assertIs(response.json()['emergency_stop_active'], False)
        gateway.request_control.assert_awaited_once_with('R01', 'CLEAR_ESTOP', {}, timeout=5.0)
        emit.assert_called_once()

    def test_pending_navigation_rejection_is_not_reported_as_applied(self):
        gateway = type('Gateway', (), {'request_control': AsyncMock(return_value={
            'ok': False, 'result': {'code': 'CLEAR_ESTOP_REJECTED_GOAL_PENDING',
                'emergency_stop_active': True, 'pre_stop_navigation_terminal': False,
                'message': 'pre-stop navigation is still canceling'},
        })})()
        with patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime.engine, 'emit') as emit:
            response = self.post_clear()
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json()['code'], 'CLEAR_ESTOP_REJECTED_GOAL_PENDING')
        self.assertIs(response.json()['emergency_stop_active'], True)
        emit.assert_not_called()

    def test_timeout_does_not_claim_clear_success(self):
        gateway = type('Gateway', (), {'request_control': AsyncMock(return_value={
            'ok': False, 'error': 'robot operation timed out',
        })})()
        with patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime.engine, 'emit') as emit:
            response = self.post_clear()
        self.assertEqual(response.status_code, 504)
        self.assertEqual(response.json()['code'], 'CLEAR_ESTOP_TIMEOUT')
        self.assertIs(response.json()['ok'], False)
        emit.assert_not_called()
