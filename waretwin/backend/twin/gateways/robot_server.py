from __future__ import annotations
from typing import Any
from .base import RobotGateway

class RobotServerGateway(RobotGateway):
    """Placeholder for: Central Django -> robot Django/Python -> ROS2.

    Intentionally has no HTTP/WebSocket implementation yet. This backend base runs
    on SimEngine only. Implement this class when the robot-server API contract is ready.
    """
    async def snapshot(self) -> dict[str, Any]:
        raise NotImplementedError('Robot Server API is not connected yet')

    async def send_command(self, robot_id: str, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        raise NotImplementedError('Robot Server API is not connected yet')
