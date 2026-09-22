import logging

from django.core.management.base import BaseCommand


log = logging.getLogger(__name__)


class Command(BaseCommand):
    help = 'Synchronize warehouse/layout master data without changing user credentials.'

    def handle(self, *args, **options):
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
            log.exception('SQLite WAL setup failed; continuing without the optional tuning')
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
            # Master-data sync is intentionally non-destructive. Keep startup
            # diagnostics visible while allowing an operator to inspect/fix the
            # database and rerun this command explicitly.
            log.exception('Warehouse master-data synchronization failed')
            self.stderr.write(self.style.WARNING(
                f'Warehouse master-data sync skipped: {type(exc).__name__}: {exc}'
            ))
