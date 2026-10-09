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

    async def test_connect_keeps_framework_cleanup_without_user_authentication(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.connect = AsyncMock()
        with patch('channels.consumer.aclose_old_connections', new_callable=AsyncMock) as cleanup:
            await consumer.dispatch({'type': 'websocket.connect'})
        cleanup.assert_awaited_once()
        consumer.connect.assert_awaited_once()

    async def test_manual_only_connection_skips_visualization_registration(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.scope = {'query_string': b'control_only=1'}
        consumer.accept = AsyncMock()
        consumer.send_json = AsyncMock()
        await consumer.connect()
        self.assertTrue(consumer.control_only)
        consumer.accept.assert_awaited_once()
        consumer.send_json.assert_awaited_once_with({'type': 'ROBOT_MANUAL_CHANNEL_READY'})
        self.assertFalse(hasattr(consumer, 'visualization_outbox'))

    async def test_full_websocket_connection_accepts_without_user_session_or_token(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.scope = {'query_string': b''}
        consumer.channel_name = 'anonymous-local-browser'
        consumer.channel_layer = AsyncMock()
        consumer.accept = AsyncMock()
        consumer.send_json = AsyncMock()
        consumer.send = AsyncMock()
        with patch('twin.consumers.runtime.ensure_started', new_callable=AsyncMock), \
                patch('twin.consumers.runtime.full_message', return_value={'type': 'FULL'}), \
                patch('twin.consumers.runtime.runtime_status_message', return_value={'type': 'RUNTIME_STATUS'}):
            await consumer.connect()
        consumer.accept.assert_awaited_once()
        consumer.send_json.assert_any_await({'type': 'FULL'})
        consumer.send_json.assert_any_await({'type': 'RUNTIME_STATUS'})
        self.assertNotIn('waretwin_user', consumer.scope)
        await consumer.disconnect(1000)

    async def test_manual_only_connection_rejects_non_manual_frames(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.control_only = True
        consumer.scope = {}
        consumer._message_window_started = 0.
        consumer._message_window_count = 0
        consumer.send_json = AsyncMock()
        with patch('twin.consumers.runtime.handle_message', new_callable=AsyncMock) as route:
            await consumer.receive_json({'type': 'ROBOT_MODE', 'robot_id': 'R01', 'mode': 'MANUAL'})
            route.assert_not_awaited()
        consumer.send_json.assert_awaited_once_with({
            'type': 'ERROR', 'code': 'CONTROL_CHANNEL_RESTRICTED',
            'message': 'manual-only channel accepts manual acquire/command frames',
        })
        with patch('twin.consumers.runtime.handle_message', new_callable=AsyncMock) as route:
            await consumer.receive_json({'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'STOP'})
            route.assert_awaited_once()

    async def test_manual_frame_keeps_original_receive_validation(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.scope = {}
        consumer._message_window_started = 0.
        consumer._message_window_count = 0
        with patch('twin.consumers.runtime.handle_message', new_callable=AsyncMock) as route:
            await consumer.dispatch({'type': 'websocket.receive',
                'text': json.dumps({'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'STOP'})})
        route.assert_awaited_once()
        self.assertEqual(route.await_args.args[1]['action'], 'STOP')
        self.assertIn('_consumer_monotonic', route.await_args.args[1])

    async def test_malformed_type_reaches_existing_error_validation(self):
        from unittest.mock import AsyncMock, patch
        consumer = TwinConsumer()
        consumer.scope = {}
        consumer._message_window_started = 0.
        consumer._message_window_count = 0
        consumer.send_json = AsyncMock()
        with patch('twin.consumers.runtime.handle_message', new_callable=AsyncMock) as route:
            await consumer.dispatch({'type': 'websocket.receive', 'text': '{"type": []}'})
        route.assert_awaited_once()
