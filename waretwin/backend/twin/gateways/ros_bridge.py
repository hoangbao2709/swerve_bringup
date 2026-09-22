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
        elif action == 'CONTROL_MODE':
            message['type'] = 'CONTROL_MODE'
        elif action == 'MANUAL_CMD':
            # Keep the wire contract self-describing.  The ROS bridge still
            # enforces its own control_mode, so this field is not a trust
            # boundary; it lets consumers/logs correlate the command with the
            # UI mode that produced it.
            message.setdefault('mode', 'MANUAL')
            message['type'] = 'MANUAL_CMD'
        elif action in ('GO_TO_TAG', 'PAUSE_TAG_NAVIGATION', 'RESUME_TAG_NAVIGATION', 'CANCEL_TAG_NAVIGATION', 'REPLAN_TAG_NAVIGATION', 'EMERGENCY_STOP', 'CLEAR_EMERGENCY_STOP'):
            message['type'] = {
                'GO_TO_TAG': 'TAG_NAV_GOAL', 'PAUSE_TAG_NAVIGATION': 'TAG_NAV_PAUSE',
                'RESUME_TAG_NAVIGATION': 'TAG_NAV_RESUME', 'CANCEL_TAG_NAVIGATION': 'TAG_NAV_CANCEL',
                'REPLAN_TAG_NAVIGATION': 'TAG_NAV_REPLAN', 'EMERGENCY_STOP': 'EMERGENCY_STOP',
                'CLEAR_EMERGENCY_STOP': 'CLEAR_EMERGENCY_STOP',
            }[action]
        else:
            raise ValueError(f'unsupported robot action: {action}')
        sent = await registry.send(message)
        return {'ok': sent, 'message': message}
