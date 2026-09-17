from datetime import timedelta

from django.contrib.auth.models import User
from django.test import TestCase
from django.utils import timezone

from twin.models import Warehouse, WarehouseMap, WarehouseOrder, RobotSchedule, WorkPoint
from twin.runtime import runtime
from twin.schedule_services import (
    create_order, create_schedule, process_runtime_schedules,
    sync_robot_profiles, sync_workpoints_from_layout,
)


class SchedulerTests(TestCase):
    def setUp(self):
        # This suite exercises the original SimEngine scheduler regardless of
        # the deployment .env used to run the tests.
        self.old_runtime_mode = runtime.runtime_mode
        runtime.runtime_mode = 'LOCAL_SIM'
        runtime.reset()
        self.user = User.objects.create_user('admin2', password='password123')
        size = runtime.layout.get('size') or {}
        self.wh = Warehouse.objects.create(
            code='WH-SCHED', name='Scheduler Test', status='ACTIVE',
            width=float(size.get('width') or 100), depth=float(size.get('depth') or 70), height=float(size.get('height') or 16),
            layout_id='wh-sched',
        )
        WarehouseMap.objects.create(warehouse=self.wh, layout=runtime.layout, draft=runtime.layout, is_active=True)
        sync_workpoints_from_layout(self.wh, runtime.layout)
        sync_robot_profiles(self.wh, runtime.engine.state['robots'])

    def tearDown(self):
        runtime.runtime_mode = self.old_runtime_mode

    def test_workpoints_cover_shelves_and_conveyors(self):
        self.assertTrue(WorkPoint.objects.filter(warehouse=self.wh, kind='SHELF').exists())
        self.assertTrue(WorkPoint.objects.filter(warehouse=self.wh, kind='CONVEYOR_IN').exists())
        self.assertTrue(WorkPoint.objects.filter(warehouse=self.wh, kind='OUTBOUND').exists())

    def test_order_schedule_and_runtime_dispatch(self):
        source = WorkPoint.objects.filter(warehouse=self.wh, kind='INBOUND').first()
        dest = WorkPoint.objects.filter(warehouse=self.wh, kind='OUTBOUND').first()
        order = create_order({'warehouse_id': self.wh.id, 'source_id': source.id, 'destination_id': dest.id, 'priority': 'HIGH'}, self.user)
        self.assertEqual(order.status, 'NEW')
        schedule = create_schedule({'order_id': order.id, 'planned_start': (timezone.now()-timedelta(seconds=1)).isoformat(), 'mode': 'AUTO'}, runtime, self.user)
        self.assertEqual(schedule.status, 'PLANNED')
        self.assertEqual(schedule.stops.count(), 2)
        changed = process_runtime_schedules(runtime.engine)
        self.assertTrue(changed)
        schedule.refresh_from_db()
        self.assertEqual(schedule.status, 'RUNNING')
        self.assertTrue(schedule.engine_task_id)
        self.assertIn(schedule.engine_task_id, runtime.engine.state['tasks'])

    def test_one_active_schedule_per_order(self):
        source = WorkPoint.objects.filter(warehouse=self.wh, kind='INBOUND').first()
        dest = WorkPoint.objects.filter(warehouse=self.wh, kind='OUTBOUND').first()
        order = create_order({'warehouse_id': self.wh.id, 'source_id': source.id, 'destination_id': dest.id}, self.user)
        create_schedule({'order_id': order.id, 'planned_start': timezone.now().isoformat()}, runtime, self.user)
        with self.assertRaises(ValueError):
            create_schedule({'order_id': order.id, 'planned_start': timezone.now().isoformat()}, runtime, self.user)
