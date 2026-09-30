"""Deterministic command-source selection for the swerve actuator input."""

from __future__ import annotations

from typing import Mapping, Optional, Tuple

TwistValues = Tuple[float, float, float]
ZERO_TWIST: TwistValues = (0.0, 0.0, 0.0)


def choose_command(
    *,
    now: float,
    timeout: float,
    mode: str,
    sources: Mapping[str, Optional[Tuple[TwistValues, float]]],
    tag_route_state: str = 'IDLE',
    emergency_stop: bool = False,
) -> tuple[str, TwistValues]:
    """Select exactly one fresh source for the robot base."""
    if emergency_stop:
        return 'ESTOP', ZERO_TWIST

    mode = str(mode or '').upper()
    if mode == 'MANUAL':
        candidates = ('WEB_MANUAL', 'DIRECT_MANUAL')
    elif mode == 'AUTONOMOUS':
        candidates = ('TAG_ROUTE',) if str(tag_route_state or '').upper() == 'APPROACH_TAG' else ('NAV2',)
    else:
        return 'NONE', ZERO_TWIST

    for owner in candidates:
        sample = sources.get(owner)
        if sample is None:
            continue
        values, received_at = sample
        if now >= received_at and now - received_at <= timeout:
            return owner, values
    return 'NONE', ZERO_TWIST
