import asyncio
import json
import threading
from unittest import IsolatedAsyncioTestCase

from asgiref.sync import sync_to_async
from twin.consumers import TwinConsumer
from twin.ros_bridge_consumer import RosBridgeConsumer


class RealtimeDispatchTests(IsolatedAsyncioTestCase):
    async def test_control_and_telemetry_survive_blocked_orm_executor(self):
        entered, release = threading.Event(), threading.Event()

        def slow_database_operation():
            entered.set()
            if not release.wait(3):
                raise RuntimeError('test ORM blocker was not released')

        blocker = asyncio.create_task(sync_to_async(
            slow_database_operation, thread_sensitive=True)())
        for _ in range(100):
            if entered.is_set():
                break
            await asyncio.sleep(.001)
        self.assertTrue(entered.is_set())
        received = []

        async def record(content):
            received.append(content)

        web, bridge = TwinConsumer(), RosBridgeConsumer()
        web.receive_json = bridge.receive_json = record
        pending = None
        try:
            for consumer, kind in ((web, 'ROBOT_MANUAL'), (web, 'ROBOT_MODE'),
                                   (bridge, 'HEARTBEAT'), (bridge, 'ROBOT_CONTROL_STATUS')):
                await asyncio.wait_for(consumer.dispatch({
                    'type': 'websocket.receive',
                    'text': json.dumps({'type': kind, 'robot_id': 'R01'}),
                }), .2)
            self.assertEqual(len(received), 4)
            # An ORM-capable command still uses the framework cleanup path.
            pending = asyncio.create_task(web.dispatch({
                'type': 'websocket.receive', 'text': json.dumps({'type': 'ASSIGN_TASK'}),
            }))
            await asyncio.sleep(.03)
            self.assertFalse(pending.done())
        finally:
            release.set()
            await blocker
            if pending is not None:
                await asyncio.wait_for(pending, 1)
        self.assertEqual(received[-1]['type'], 'ASSIGN_TASK')

    async def test_connect_authentication_keeps_framework_cleanup(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.connect = AsyncMock()
        with patch('channels.consumer.aclose_old_connections', new_callable=AsyncMock) as cleanup:
            await consumer.dispatch({'type': 'websocket.connect'})
        cleanup.assert_awaited_once()
        consumer.connect.assert_awaited_once()

    async def test_manual_frame_keeps_original_receive_validation(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.scope = {'waretwin_user': object()}
        consumer._message_window_started = 0.
        consumer._message_window_count = 0
        with patch('twin.consumers.runtime.handle_message', new_callable=AsyncMock) as route:
            await consumer.dispatch({'type': 'websocket.receive',
                'text': json.dumps({'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'STOP'})})
        route.assert_awaited_once()
        self.assertEqual(route.await_args.args[1]['action'], 'STOP')
        self.assertIn('_consumer_monotonic', route.await_args.args[1])
