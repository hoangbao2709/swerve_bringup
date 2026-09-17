from copy import deepcopy
from datetime import timedelta
from unittest import TestCase

from django.test import TestCase as DjangoTestCase, override_settings
from django.utils import timezone

from twin.models import RobotProfile, RobotSchedule, ScheduleStop, Warehouse, WarehouseMap, WarehouseOrder, WorkPoint
from twin.runtime import runtime
from twin.schedule_services import apply_external_nav_status, prepare_external_dispatches


class ExternalRuntimeTests(TestCase):
    @override_settings(WARETWIN_RUNTIME_MODE='GAZEBO_ROS')
    def test_external_runtime_never_uses_local_sim_step(self):
        old_mode = runtime.runtime_mode
        old_step = runtime.engine.step
        runtime.runtime_mode = 'GAZEBO_ROS'
        before = runtime.engine.state['sim']['tick']
        try:
            runtime.engine.step = lambda: self.fail('SimEngine.step must not run in external mode')
            runtime.engine.state['sim']['tick'] = before + 1
            self.assertEqual(runtime.engine.state['sim']['tick'], before + 1)
        finally:
            runtime.engine.step = old_step
            runtime.runtime_mode = old_mode


class ExternalStopLifecycleTests(DjangoTestCase):
    def setUp(self):
        self.old_connected = runtime.ros_bridge_connected
        self.old_robot = deepcopy(runtime.engine.state['robots'].get('R01'))
        self.wh = Warehouse.objects.create(
            code='WH-ROS-STOP', name='ROS stop test', status='ACTIVE',
            width=30, depth=30, height=5, layout_id='ros-stop-test',
        )
        WarehouseMap.objects.create(
            warehouse=self.wh, layout={}, draft={}, is_active=True,
        )
        self.source = WorkPoint.objects.create(
            warehouse=self.wh, code='INBOUND-01', name='Inbound', kind='INBOUND',
            x=1.0, y=2.0, yaw=0.1,
        )
        self.destination = WorkPoint.objects.create(
            warehouse=self.wh, code='SHELF-A101', name='Shelf', kind='SHELF',
            x=8.0, y=9.0, yaw=0.2,
        )
        self.robot = RobotProfile.objects.create(
            warehouse=self.wh, robot_id='R01', name='R01', enabled=True,
        )
        self.order = WarehouseOrder.objects.create(
            warehouse=self.wh, order_no='ORD-ROS-1', type='MOVE', priority='NORMAL',
            source=self.source, destination=self.destination, load_units=1,
        )
        self.schedule = RobotSchedule.objects.create(
            schedule_id='SCH-ROS-1', warehouse=self.wh, order=self.order,
            robot=self.robot, status='PLANNED',
            planned_start=timezone.now() - timedelta(seconds=1),
        )
        self.source_stop = ScheduleStop.objects.create(
            schedule=self.schedule, sequence=0, workpoint=self.source,
            action='PICKUP', service_seconds=0,
        )
        self.destination_stop = ScheduleStop.objects.create(
            schedule=self.schedule, sequence=1, workpoint=self.destination,
            action='DROPOFF', service_seconds=0,
        )
        runtime.ros_bridge_connected = True
        runtime.engine.state['robots']['R01'] = {
            'id': 'R01', 'status': 'ACTIVE', 'fsm': 'IDLE', 'battery': 100.0,
            'floor': 1, 'position': [0.0, 0.0, 0.0],
            'load': {'current': 0, 'capacity': 4},
        }

    def tearDown(self):
        runtime.ros_bridge_connected = self.old_connected
        if self.old_robot is None:
            runtime.engine.state['robots'].pop('R01', None)
        else:
            runtime.engine.state['robots']['R01'] = self.old_robot

    def test_source_and_destination_are_independent_nav2_goals(self):
        goals, changed = prepare_external_dispatches(runtime)
        self.assertTrue(changed)
        self.assertEqual(len(goals), 1)
        self.assertEqual(goals[0]['stop_id'], self.source_stop.id)
        self.assertEqual((goals[0]['x'], goals[0]['y']), (1.0, 2.0))

        # A result for any stop except the exact active stop_id is ignored.
        self.assertFalse(apply_external_nav_status({
            'type': 'NAV_STATUS', 'robot_id': 'R01',
            'schedule_id': self.schedule.schedule_id,
            'stop_id': self.destination_stop.id, 'status': 'SUCCEEDED',
        }))

        self.assertTrue(apply_external_nav_status({
            'type': 'NAV_STATUS', 'robot_id': 'R01',
            'schedule_id': self.schedule.schedule_id,
            'stop_id': self.source_stop.id, 'status': 'SUCCEEDED',
        }))
        self.source_stop.refresh_from_db()
        self.destination_stop.refresh_from_db()
        self.schedule.refresh_from_db()
        self.assertEqual(self.source_stop.status, 'ARRIVED')
        self.assertEqual(self.destination_stop.status, 'PENDING')
        self.assertEqual(self.schedule.current_leg, 0)

        goals, _ = prepare_external_dispatches(runtime)
        self.source_stop.refresh_from_db()
        self.destination_stop.refresh_from_db()
        self.schedule.refresh_from_db()
        self.assertEqual(self.source_stop.status, 'COMPLETED')
        self.assertEqual(self.destination_stop.status, 'ACTIVE')
        self.assertEqual(self.schedule.current_leg, 1)
        self.assertEqual(goals[0]['stop_id'], self.destination_stop.id)

        self.assertTrue(apply_external_nav_status({
            'type': 'NAV_STATUS', 'robot_id': 'R01',
            'schedule_id': self.schedule.schedule_id,
            'stop_id': self.destination_stop.id, 'status': 'SUCCEEDED',
        }))
        self.destination_stop.refresh_from_db()
        self.schedule.refresh_from_db()
        self.assertEqual(self.destination_stop.status, 'ARRIVED')
        self.assertNotEqual(self.schedule.status, 'COMPLETED')

        goals, _ = prepare_external_dispatches(runtime)
        self.assertEqual(goals, [])
        self.destination_stop.refresh_from_db()
        self.schedule.refresh_from_db()
        self.order.refresh_from_db()
        self.assertEqual(self.destination_stop.status, 'COMPLETED')
        self.assertEqual(self.schedule.status, 'COMPLETED')
        self.assertEqual(self.order.status, 'COMPLETED')
