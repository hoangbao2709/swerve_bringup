import json

from django.contrib.auth.models import User
from django.test import Client, TestCase


class RobotControlAuthenticationTests(TestCase):
    def setUp(self):
        self.user = User.objects.create_user(
            username='control-operator', email='operator@example.test', password='test-password-123'
        )
        self.client = Client()

    def test_login_me_logout_and_expired_token_contract(self):
        login = self.client.post('/api/auth/login', data=json.dumps({
            'username': 'control-operator', 'password': 'test-password-123',
        }), content_type='application/json')
        self.assertEqual(login.status_code, 200)
        payload = login.json()
        self.assertTrue(payload['access_token'])
        self.assertEqual(payload['token_type'], 'bearer')
        self.assertEqual(payload['user']['username'], 'control-operator')

        headers = {'HTTP_AUTHORIZATION': f"Bearer {payload['access_token']}"}
        current = self.client.get('/api/auth/me', **headers)
        self.assertEqual(current.status_code, 200)
        self.assertEqual(current.json()['id'], self.user.id)

        logout = self.client.post('/api/auth/logout', **headers)
        self.assertEqual(logout.status_code, 200)
        self.assertTrue(logout.json()['ok'])
        self.assertEqual(self.client.get('/api/auth/me', **headers).status_code, 401)

    def test_invalid_credentials_fail_and_public_registration_is_not_exposed(self):
        invalid = self.client.post('/api/auth/login', data=json.dumps({
            'username': 'control-operator', 'password': 'wrong-password',
        }), content_type='application/json')
        self.assertEqual(invalid.status_code, 401)

        registration = self.client.post('/api/auth/register', data='{}', content_type='application/json')
        self.assertEqual(registration.status_code, 404)
