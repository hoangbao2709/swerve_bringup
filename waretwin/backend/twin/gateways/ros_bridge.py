from __future__ import annotations

from typing import Any

from .base import RobotGateway
from ..ros_bridge_consumer import registry


class RosBridgeGateway(RobotGateway):
    """Django-side command boundary; it knows no ROS implementation details."""

    async def snapshot(self) -> dict[str, Any]:
        return {}

    async def send_command(self, robot_id: str, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        message = dict(payload)
        message.update({'robot_id': robot_id})
        if action == 'NAVIGATE':
            message['type'] = 'NAV_GOAL'
        elif action == 'CANCEL_NAVIGATION':
            message['type'] = 'CANCEL_NAVIGATION'
        elif action == 'STOP':
            message['type'] = 'CANCEL_NAVIGATION'
        else:
            raise ValueError(f'unsupported robot action: {action}')
        sent = await registry.send(message)
        return {'ok': sent, 'message': message}
