from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any

class RobotGateway(ABC):
    """Future LIVE-mode boundary between Central Django and robot-side servers."""
    @abstractmethod
    async def snapshot(self) -> dict[str, Any]: ...

    @abstractmethod
    async def send_command(self, robot_id: str, action: str, payload: dict[str, Any]) -> dict[str, Any]: ...
