import os

from django.contrib.auth.models import User
from django.core.management.base import BaseCommand, CommandError

from accounts.models import ensure_profile


class Command(BaseCommand):
    help = 'Ensure the configured local WareTwin admin exists without resetting an existing password.'

    def handle(self, *args, **options):
        username = os.getenv('TWIN_ADMIN_USERNAME', 'admin').strip() or 'admin'
        email = os.getenv('TWIN_ADMIN_EMAIL', 'admin@example.com').strip() or 'admin@example.com'
        password = os.getenv('TWIN_ADMIN_PASSWORD', '').strip()
        user = User.objects.filter(username=username).first()
        created = user is None

        if created:
            if not password or password in {'admin12345', 'change-me-before-first-run'}:
                raise CommandError(
                    'TWIN_ADMIN_PASSWORD must be a non-default value when the configured admin does not exist; '
                    'set it in waretwin/backend/.env before running seed_demo'
                )
            user = User(username=username, email=email)
            user.set_password(password)
        else:
            # Never call set_password for an existing account. A backend restart
            # or a repeated setup must not invalidate an operator's password.
            user.email = email

        user.is_active = True
        user.is_staff = True
        user.is_superuser = True
        if created:
            user.save()
        else:
            user.save(update_fields=['email', 'is_active', 'is_staff', 'is_superuser'])
        ensure_profile(user, 'admin')

        if created:
            self.stdout.write(self.style.SUCCESS(f'Admin created: {username} (password initialized from TWIN_ADMIN_PASSWORD)'))
        else:
            self.stdout.write(self.style.SUCCESS(f'Admin ensured: {username} (existing password preserved)'))
