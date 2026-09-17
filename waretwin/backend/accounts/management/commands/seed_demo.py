import os
from django.contrib.auth.models import User
from django.core.management.base import BaseCommand
from accounts.models import ensure_profile

class Command(BaseCommand):
    help = 'Create/update local WareTwin demo admin.'

    def handle(self, *args, **options):
        username = os.getenv('TWIN_ADMIN_USERNAME', 'admin')
        email = os.getenv('TWIN_ADMIN_EMAIL', 'admin@example.com')
        password = os.getenv('TWIN_ADMIN_PASSWORD', 'admin12345')
        user, _ = User.objects.get_or_create(username=username, defaults={'email': email})
        user.email = email
        user.is_active = True
        user.is_staff = True
        user.is_superuser = True
        user.set_password(password)
        user.save()
        ensure_profile(user, 'admin')
        self.stdout.write(self.style.SUCCESS(f'Admin ready: {username} / {password} (development only; change it before deployment)'))

        # The dashboard performs concurrent reads while the scheduler occasionally
        # persists status transitions. WAL lets readers proceed during a writer and
        # is much more suitable than SQLite's default rollback journal for ASGI dev.
        try:
            from django.db import connection
            if connection.vendor == 'sqlite':
                with connection.cursor() as cursor:
                    cursor.execute('PRAGMA journal_mode=WAL;')
                    cursor.execute('PRAGMA synchronous=NORMAL;')
                    cursor.execute('PRAGMA busy_timeout=30000;')
                self.stdout.write(self.style.SUCCESS('SQLite configured: WAL + busy_timeout=30000ms'))
        except Exception as exc:
            self.stderr.write(self.style.WARNING(f'SQLite WAL setup skipped: {exc}'))
        try:
            from twin.runtime import runtime
            from twin.models import WarehouseMap
            from twin.warehouse_services import sync_from_layout, ensure_active_map
            if WarehouseMap.objects.exists():
                active = ensure_active_map(runtime.layout)
                result = sync_from_layout(active.layout, warehouse=active.warehouse, prune=False)
            else:
                result = sync_from_layout(runtime.layout)
                active = ensure_active_map(runtime.layout)
            from twin.schedule_services import sync_workpoints_from_layout, sync_robot_profiles, sync_inventory_placeholders
            wp = sync_workpoints_from_layout(active.warehouse, active.layout, prune=False)
            rp = sync_robot_profiles(active.warehouse, runtime.engine.state.get('robots') or {})
            inv = sync_inventory_placeholders(active.warehouse)
            self.stdout.write(self.style.SUCCESS(
                f"Warehouse master data synced: {result['warehouses']} warehouse, {result['zones']} zones, {result['shelves']} shelves; "
                f"workpoints={wp['total']}, robots={rp['total']}, inventory_backfill={inv['created']}; active map warehouse_id={active.warehouse_id} revision={active.revision}"
            ))
        except Exception as exc:
            self.stderr.write(self.style.WARNING(f'Warehouse master-data sync skipped: {exc}'))
