import secrets
from django.contrib.auth.models import User
from django.db import models

class UserProfile(models.Model):
    ROLE_CHOICES = [('admin', 'Admin'), ('user', 'User')]
    user = models.OneToOneField(User, on_delete=models.CASCADE, related_name='waretwin_profile')
    role = models.CharField(max_length=16, choices=ROLE_CHOICES, default='user')

class ApiToken(models.Model):
    key = models.CharField(max_length=128, unique=True, db_index=True)
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name='waretwin_api_tokens')
    created_at = models.DateTimeField(auto_now_add=True)

    @classmethod
    def issue(cls, user):
        cls.objects.filter(user=user).delete()
        return cls.objects.create(user=user, key=secrets.token_urlsafe(48))

def role_of(user):
    try:
        return user.waretwin_profile.role
    except UserProfile.DoesNotExist:
        return 'admin' if user.is_superuser else 'user'

def public_user(user):
    return {
        'id': user.id,
        'username': user.username,
        'email': user.email,
        'role': role_of(user),
        'is_active': user.is_active,
    }

def ensure_profile(user, role='user'):
    profile, _ = UserProfile.objects.get_or_create(user=user, defaults={'role': role})
    if profile.role != role:
        profile.role = role
        profile.save(update_fields=['role'])
    return profile
