from copy import deepcopy
from datetime import timedelta
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import AsyncMock, Mock, patch

from asgiref.sync import async_to_sync
from django.test import TestCase as DjangoTestCase, override_settings
from django.utils import timezone

from twin.models import RobotProfile, RobotSchedule, ScheduleStop, Warehouse, WarehouseMap, WarehouseOrder, WorkPoint
from twin.runtime import runtime
from twin.sim.engine import SimEngine


class ExternalRuntimeTests(TestCase):
    @override_settings(WARETWIN_RUNTIME_MODE='GAZEBO_ROS')
    def test_external_runtime_starts_without_seeded_robot_or_device_state(self):
        old_mode = runtime.runtime_mode
        old_engine = runtime.engine
        runtime.runtime_mode = 'GAZEBO_ROS'
        try:
            runtime.engine = SimEngine(runtime.layout)
            runtime.engine.state['conveyors'] = {'CV-FAKE': {'id': 'CV-FAKE', 'status': 'RUNNING'}}
            runtime._prepare_external_cache()
            state = runtime.full_message()['state']
            self.assertEqual(state['robots'], {})
            self.assertEqual(state['conveyors'], {})
            self.assertEqual(state['tasks'], {})
            self.assertEqual(state['kpi']['fleet']['total'], 0)
        finally:
            runtime.engine = old_engine
            runtime.runtime_mode = old_mode


class ExternalStopLifecycleTests(DjangoTestCase):
    def setUp(self):
        self.old_connected = runtime.ros_bridge_connected
        self.old_robot = deepcopy(runtime.engine.state['robots'].get('R01'))
        self.old_mode = runtime.runtime_mode
        self.old_tick = runtime.engine.state['sim']['tick']
        self.old_client_count = runtime.client_count
        self.old_gateway = runtime.robot_gateway
        self.old_last_telemetry = runtime.last_telemetry_at
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
        runtime.runtime_mode = self.old_mode
        runtime.engine.state['sim']['tick'] = self.old_tick
        runtime.client_count = self.old_client_count
        runtime.robot_gateway = self.old_gateway
        runtime.last_telemetry_at = self.old_last_telemetry
        if self.old_robot is None:
            runtime.engine.state['robots'].pop('R01', None)
        else:
            runtime.engine.state['robots']['R01'] = self.old_robot

    def test_stale_database_order_and_schedule_never_dispatch_navigation(self):
        runtime.runtime_mode = 'GAZEBO_ROS'
        runtime.engine.state['sim']['tick'] = 10
        runtime.client_count = 0
        runtime.last_telemetry_at = None
        runtime.robot_gateway = SimpleNamespace(
            send_command=AsyncMock(return_value={'ok': True}),
        )
        goal = {
            'robot_id': 'R01', 'schedule_id': self.schedule.schedule_id,
            'stop_id': self.source_stop.id, 'frame_id': 'map',
            'x': self.source.x, 'y': self.source.y, 'yaw': self.source.yaw,
        }
        # The legacy scheduler would return this due, stale DB goal. External
        # runtime ticks must not call the scheduler or forward any NAVIGATE.
        with patch('twin.schedule_services.prepare_external_dispatches', Mock(return_value=([goal], True))) as dispatch:
            with patch.object(runtime, 'runtime_status_message', return_value={'map_sync_status': 'SYNCED'}):
                async_to_sync(runtime.after_ticks)()
        dispatch.assert_not_called()
        runtime.robot_gateway.send_command.assert_not_awaited()
        self.schedule.refresh_from_db()
        self.order.refresh_from_db()
        self.assertEqual(self.schedule.status, 'PLANNED')
        self.assertEqual(self.order.status, 'NEW')
        self.assertEqual(self.source_stop.status, 'PENDING')
