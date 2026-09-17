"""PLC boundary for conveyor control.

The simulator intentionally exposes the same small contract a Modbus/OPC-UA
adapter will expose later. Scheduler code submits transport requests; it never
starts motors or moves items directly.
"""
from __future__ import annotations

import copy
import uuid
from dataclasses import dataclass, field
from typing import Any, Protocol


class ConveyorPLCAdapter(Protocol):
    def tick(self, dt_s: float) -> None: ...
    def command(self, conveyor_id: str, action: str, **kwargs: Any) -> dict[str, Any]: ...
    def snapshot(self) -> dict[str, dict[str, Any]]: ...
    def request_handshake(self, conveyor_id: str, robot_id: str, item_id: str, phase: str) -> dict[str, Any]: ...


@dataclass
class BeltItem:
    item_id: str
    order_id: str | None = None
    status: str = "ON_BELT"
    position_m: float = 0.0
    load_units: int = 1
    handshake: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "item_id": self.item_id,
            "order_id": self.order_id,
            "status": self.status,
            "position_m": round(self.position_m, 3),
            "load_units": self.load_units,
            "handshake": dict(self.handshake),
        }


class PLCSimulator:
    """Deterministic conveyor PLC with sensors, faults and item tracking."""

    def __init__(self, layout: dict[str, Any]) -> None:
        self.layout = layout
        self._belts: dict[str, dict[str, Any]] = {}
        for conveyor in layout.get("conveyors", []):
            cid = str(conveyor["id"])
            length = self._length(conveyor)
            self._belts[cid] = {
                "id": cid,
                "mode": "RUNNING",
                "status": "RUNNING",
                "speed_mps": float(conveyor.get("speed_mps", 0.5)),
                "configured_speed_mps": float(conveyor.get("speed_mps", 0.5)),
                "length_m": round(length, 3),
                "sensors": {"entry": False, "exit": False, "jam": False, "blocked": False},
                "fault_code": None,
                "emergency_stop": False,
                "maintenance": False,
                "items": [
                    BeltItem(item_id=f"{cid}-BOOT-{n + 1}", position_m=length * (0.2 + n * 0.18)).as_dict()
                    for n in range(4)
                ],
                "throughput_per_min": 0.0,
                "items_on_belt": 4,
                "last_command": "BOOT",
            }

    @staticmethod
    def _length(conveyor: dict[str, Any]) -> float:
        path = conveyor.get("path") or []
        return sum(((path[i + 1][0] - path[i][0]) ** 2 + (path[i + 1][1] - path[i][1]) ** 2) ** 0.5 for i in range(len(path) - 1)) or 1.0

    def _belt(self, conveyor_id: str) -> dict[str, Any]:
        if conveyor_id not in self._belts:
            raise KeyError(f"Unknown conveyor {conveyor_id}")
        return self._belts[conveyor_id]

    def tick(self, dt_s: float) -> None:
        for belt in self._belts.values():
            active = belt["mode"] == "RUNNING" and belt["status"] == "RUNNING" and not belt["emergency_stop"] and not belt["maintenance"]
            belt["sensors"]["entry"] = False
            belt["sensors"]["exit"] = False
            if active:
                for item in belt["items"]:
                    if item["status"] not in ("AT_ENTRY", "ON_BELT", "HANDSHAKE_READY"):
                        continue
                    if item["status"] == "AT_ENTRY":
                        item["status"] = "ON_BELT"
                    item["position_m"] += belt["speed_mps"] * dt_s
                    if item["position_m"] >= belt["length_m"]:
                        item["position_m"] = belt["length_m"]
                        item["status"] = "AT_EXIT"
                        belt["sensors"]["exit"] = True
            belt["sensors"]["blocked"] = any(item["status"] == "AT_EXIT" for item in belt["items"])
            belt["sensors"]["jam"] = belt["mode"] == "FAULT" and belt["fault_code"] == "JAM"
            belt["items_on_belt"] = sum(1 for item in belt["items"] if item["status"] in ("ON_BELT", "HANDSHAKE_READY", "AT_EXIT"))
            belt["throughput_per_min"] = round(sum(1 for item in belt["items"] if item["status"] == "AT_EXIT") * 60 / max(belt["length_m"] / max(belt["configured_speed_mps"], 0.01), 1), 1)

    def command(self, conveyor_id: str, action: str, **kwargs: Any) -> dict[str, Any]:
        belt = self._belt(conveyor_id)
        action = action.upper()
        if action == "START":
            if belt["emergency_stop"] or belt["fault_code"]:
                raise ValueError("Clear emergency stop/fault before starting")
            belt["maintenance"] = False
            belt["mode"] = belt["status"] = "RUNNING"
        elif action == "STOP":
            belt["mode"] = belt["status"] = "STOPPED"
        elif action == "MAINTENANCE":
            belt["maintenance"] = True
            belt["mode"] = belt["status"] = "MAINTENANCE"
        elif action == "EMERGENCY_STOP":
            belt["emergency_stop"] = True
            belt["mode"] = belt["status"] = "STOPPED"
            belt["fault_code"] = "EMERGENCY_STOP"
        elif action == "CLEAR_FAULT":
            belt["fault_code"] = None
            belt["emergency_stop"] = False
            belt["sensors"]["jam"] = False
            belt["status"] = "STOPPED"
            belt["mode"] = "STOPPED"
        elif action == "FAULT":
            belt["fault_code"] = str(kwargs.get("fault_code") or "PLC_FAULT")
            belt["mode"] = "FAULT"
            belt["status"] = "ERROR"
        elif action == "SET_SPEED":
            speed = max(0.0, min(float(kwargs.get("speed_mps", belt["speed_mps"])), 5.0))
            belt["speed_mps"] = speed
        elif action == "ENQUEUE":
            item_id = str(kwargs.get("item_id") or f"{conveyor_id}-{uuid.uuid4().hex[:8]}")
            position = max(0.0, min(float(kwargs.get("position_m", 0.0)), belt["length_m"]))
            item = BeltItem(item_id=item_id, order_id=kwargs.get("order_id"), load_units=int(kwargs.get("load_units", 1)), position_m=position).as_dict()
            item["status"] = "AT_ENTRY" if position <= 0 else "AT_EXIT" if position >= belt["length_m"] else "ON_BELT"
            belt["items"].append(item)
            belt["sensors"]["entry"] = True
        elif action == "RELEASE_EXIT":
            item_id = str(kwargs.get("item_id") or "")
            for item in belt["items"]:
                if item["item_id"] == item_id and item["status"] == "AT_EXIT":
                    item["status"] = "RELEASED"
                    item["handshake"] = {}
                    break
        else:
            raise ValueError(f"Unsupported conveyor action: {action}")
        belt["last_command"] = action
        return copy.deepcopy(belt)

    def request_handshake(self, conveyor_id: str, robot_id: str, item_id: str, phase: str) -> dict[str, Any]:
        belt = self._belt(conveyor_id)
        item = next((x for x in belt["items"] if x["item_id"] == item_id), None)
        if item is None:
            raise ValueError("Conveyor item not found")
        phase = phase.upper()
        if phase == "REQUEST_PICKUP" and item["status"] != "AT_EXIT":
            raise ValueError("Item is not ready at conveyor exit")
        if phase == "REQUEST_DROP" and item["status"] != "AT_ENTRY":
            raise ValueError("Conveyor entry sensor is not ready")
        if phase not in ("REQUEST_PICKUP", "PICKUP_CONFIRMED", "DELIVERY_CONFIRMED", "REQUEST_DROP", "DROP_CONFIRMED"):
            raise ValueError("Invalid handshake phase")
        item["handshake"] = {"robot_id": robot_id, "phase": phase, "token": item["handshake"].get("token") or uuid.uuid4().hex}
        if phase == "PICKUP_CONFIRMED":
            item["status"] = "HANDED_TO_ROBOT"
        if phase == "DROP_CONFIRMED":
            item["status"] = "ON_BELT"
        if phase == "DELIVERY_CONFIRMED":
            item["status"] = "DELIVERED"
        return copy.deepcopy(item)

    def snapshot(self) -> dict[str, dict[str, Any]]:
        return copy.deepcopy(self._belts)
