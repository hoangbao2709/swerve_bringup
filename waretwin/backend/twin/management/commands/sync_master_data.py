import logging

from django.core.management.base import BaseCommand, CommandError
from django.db import DatabaseError, transaction


log = logging.getLogger(__name__)


class Command(BaseCommand):
    help = 'Synchronize warehouse/layout master data.'

    def handle(self, *args, **options):
        # The dashboard performs concurrent reads while the scheduler occasionally
        # persists status transitions. WAL lets readers proceed during a writer and
        # is more suitable than SQLite's default rollback journal for ASGI dev.
        try:
            from django.db import connection

            if connection.vendor == 'sqlite' and not connection.in_atomic_block:
                with connection.cursor() as cursor:
                    cursor.execute('PRAGMA journal_mode=WAL;')
                    cursor.execute('PRAGMA synchronous=NORMAL;')
                    cursor.execute('PRAGMA busy_timeout=30000;')
                self.stdout.write(self.style.SUCCESS('SQLite configured: WAL + busy_timeout=30000ms'))
        except DatabaseError as exc:
            log.warning('Optional SQLite WAL tuning failed; continuing without it: %s', type(exc).__name__)
            self.stderr.write(self.style.WARNING(f'SQLite WAL setup skipped: {type(exc).__name__}: {exc}'))

        try:
            from twin.runtime import runtime
            from twin.models import WarehouseMap
            from twin.schedule_services import (
                sync_inventory_placeholders,
                sync_robot_profiles,
                sync_workpoints_from_layout,
            )
            from twin.warehouse_services import ensure_active_map, sync_from_layout

            # The active warehouse/map and its scheduler catalog form one
            # startup consistency boundary. Never report success after a
            # partial sync: navigation must not become available against a
            # mixed master-data revision.
            with transaction.atomic():
                if WarehouseMap.objects.exists():
                    active = ensure_active_map(runtime.layout)
                    result = sync_from_layout(active.layout, warehouse=active.warehouse, prune=False)
                else:
                    result = sync_from_layout(runtime.layout)
                    active = ensure_active_map(runtime.layout)
                workpoints = sync_workpoints_from_layout(active.warehouse, active.layout, prune=False)
                robots = sync_robot_profiles(active.warehouse, runtime.engine.state.get('robots') or {})
                inventory = sync_inventory_placeholders(active.warehouse)
            self.stdout.write(self.style.SUCCESS(
                f"Warehouse master data synced: {result['warehouses']} warehouse, "
                f"{result['zones']} zones, {result['shelves']} shelves; "
                f"workpoints={workpoints['total']}, robots={robots['total']}, "
                f"inventory_backfill={inventory['created']}; "
                f"active map warehouse_id={active.warehouse_id} revision={active.revision}"
            ))
        except Exception as exc:
            log.exception('Critical warehouse master-data synchronization failed')
            raise CommandError(
                f'critical warehouse master-data synchronization failed: {type(exc).__name__}; '
                'backend/robot startup must remain unavailable until corrected'
            ) from exc
