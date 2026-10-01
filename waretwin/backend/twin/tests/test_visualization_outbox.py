import asyncio
import json
from unittest import IsolatedAsyncioTestCase
from twin.visualization_outbox import VisualizationOutbox


class VisualizationOutboxTests(IsolatedAsyncioTestCase):
    async def test_acceleration_keeps_utf8_validation_and_fragmented_masking(self):
        from autobahn.websocket.utf8validator import Utf8Validator
        from autobahn.websocket.xormasker import create_xor_masker
        self.assertTrue(Utf8Validator.__module__.startswith('wsaccel'))
        self.assertTrue(create_xor_masker.__module__.startswith('wsaccel'))
        validator = Utf8Validator()
        self.assertTrue(validator.validate('LiDAR bản đồ'.encode())[0])
        validator.reset()
        self.assertFalse(validator.validate(b'\xff')[0])
        mask, data = b'abcd', bytes(range(256)) * 8
        masker = create_xor_masker(mask, len(data))
        self.assertEqual(masker.process(data[:7]) + masker.process(data[7:]),
            bytes(value ^ mask[index % 4] for index, value in enumerate(data)))

    async def test_browser_receipt_bounds_transport_and_ack_bypasses_it(self):
        frames = []
        async def send(**data): frames.append(json.loads(data['text_data']))
        outbox = VisualizationOutbox(send)
        outbox.delivery_ack = True
        try:
            outbox.offer({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'revision': 1})
            for _ in range(100):
                if frames: break
                await asyncio.sleep(.01)
            for revision in range(2, 1000):
                outbox.offer({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'revision': revision})
            await asyncio.sleep(.03)
            self.assertEqual(len(frames), 1)
            outbox.offer({'type': 'ROBOT_DETAIL_VIEW_STATUS', 'robot_id': 'R01'})
            for _ in range(100):
                if len(frames) == 2: break
                await asyncio.sleep(.01)
            self.assertEqual(frames[-1]['type'], 'ROBOT_DETAIL_VIEW_STATUS')
            outbox.offer({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'revision': 1000})
            outbox.acknowledge(999999)
            self.assertIsNotNone(outbox.inflight)
            outbox.acknowledge(frames[0]['visualization_delivery_id'])
            for _ in range(100):
                if len(frames) == 3: break
                await asyncio.sleep(.01)
            self.assertEqual(frames[-1]['revision'], 1000)
        finally:
            await outbox.close()

    async def test_slow_browser_keeps_only_latest_frame_and_prioritizes_ack(self):
        sending, release = asyncio.Event(), asyncio.Event()
        frames = []
        async def send(**data):
            frames.append(json.loads(data['text_data']))
            if len(frames) == 1:
                sending.set(); await release.wait()
        outbox = VisualizationOutbox(send)
        try:
            outbox.offer({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'revision': 0})
            await asyncio.wait_for(sending.wait(), 2)
            for revision in range(1, 1000):
                outbox.offer({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'revision': revision,
                    'view_epoch': 2, 'bridge_epoch': 'bridge'})
            outbox.offer({'type': 'ROBOT_DETAIL_VIEW_STATUS', 'robot_id': 'R01',
                'view_epoch': 2, 'bridge_epoch': 'bridge'})
            outbox.offer({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'revision': 1000,
                'view_epoch': 2, 'bridge_epoch': 'bridge'})
            self.assertLessEqual(len(outbox.pending), 2)
            release.set()
            for _ in range(100):
                if len(frames) >= 3: break
                await asyncio.sleep(.01)
            self.assertEqual([frame['type'] for frame in frames], ['LIDAR_MAP_3D', 'ROBOT_DETAIL_VIEW_STATUS', 'LIDAR_MAP_3D'])
            self.assertEqual(frames[-1]['revision'], 1000)
            self.assertGreaterEqual(outbox.dropped, 998)
        finally:
            release.set(); await outbox.close()

    async def test_old_epoch_and_many_robots_never_make_unbounded_queue(self):
        async def send(**data): pass
        outbox = VisualizationOutbox(send, capacity=8)
        try:
            for i in range(1000):
                outbox.offer({'type': 'LIDAR_MAP_3D', 'robot_id': str(i)})
            self.assertEqual(len(outbox.pending), 8)
            outbox.offer({'type': 'ROBOT_DETAIL_VIEW_STATUS', 'robot_id': 'R01', 'view_epoch': 4, 'bridge_epoch': 'new'})
            self.assertFalse(outbox.current({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'view_epoch': 3, 'bridge_epoch': 'new'}))
            self.assertFalse(outbox.current({'type': 'LIDAR_MAP_3D', 'robot_id': 'R01', 'view_epoch': 4, 'bridge_epoch': 'old'}))
        finally:
            await outbox.close()
