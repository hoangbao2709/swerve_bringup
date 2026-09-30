from unittest.mock import AsyncMock, patch
from django.test import SimpleTestCase
from twin.runtime import runtime
from twin.consumers import TwinConsumer
from twin.gateways.ros_bridge import RosBridgeGateway


class ControlModeHandshakeTests(SimpleTestCase):
    async def test_disconnect_stops_only_robots_owned_by_that_client(self):
        previous=getattr(runtime,'manual_owners',{})
        runtime.manual_owners={'R01':'owner','R02':'other'}
        consumer=TwinConsumer();consumer.channel_name='owner'
        consumer.channel_layer=type('Layer',(),{'group_discard':AsyncMock()})()
        gateway=type('Gateway',(),{'send_command':AsyncMock(return_value={'ok':True})})()
        try:
            with patch.object(runtime,'gateway',return_value=gateway):await consumer.disconnect(1000)
            gateway.send_command.assert_awaited_once_with('R01','MANUAL_DISCONNECT',{})
            self.assertEqual(runtime.manual_owners,{'R02':'other'})
        finally:runtime.manual_owners=previous

    async def test_disconnect_wire_message_is_a_barrier_not_navigation_cancel(self):
        with patch('twin.gateways.ros_bridge.registry.send',new=AsyncMock(return_value=True)) as send:
            await RosBridgeGateway().send_command('R01','MANUAL_DISCONNECT',{})
            send.assert_awaited_once_with({'robot_id':'R01','type':'MANUAL_DISCONNECT'})

    async def test_stale_applied_confirmation_is_not_broadcast(self):
        previous=getattr(runtime,'control_mode_requests',{})
        runtime.control_mode_requests={'R01':{'request_id':'new'}}
        try:
            with patch.object(runtime,'broadcast',new=AsyncMock()) as broadcast:
                await runtime.handle_ros_message({'type':'ROBOT_CONTROL_STATUS','robot_id':'R01','request_id':'old','mode_transition_state':'APPLIED','accepted':True})
                broadcast.assert_not_awaited()
        finally:runtime.control_mode_requests=previous
