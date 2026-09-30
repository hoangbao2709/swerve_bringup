"""Focused tests for production swerve command-source ownership rules."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'swerve_controller'))

from command_ownership import choose_command


def test_web_manual_lease_preempts_direct_manual_and_zero_is_a_stop():
    owner, command = choose_command(
        now=10.0,
        timeout=0.5,
        mode='MANUAL',
        sources={
            'WEB_MANUAL': ((0.0, 0.0, 0.0), 9.9),
            'DIRECT_MANUAL': ((0.25, 0.0, 0.0), 10.0),
        },
    )
    assert owner == 'WEB_MANUAL'
    assert command == (0.0, 0.0, 0.0)


def test_direct_ros_manual_is_selected_after_web_lease_expires():
    owner, command = choose_command(
        now=10.6,
        timeout=0.5,
        mode='MANUAL',
        sources={
            'WEB_MANUAL': ((0.0, 0.0, 0.0), 10.0),
            'DIRECT_MANUAL': ((0.25, 0.0, 0.0), 10.4),
        },
    )
    assert owner == 'DIRECT_MANUAL'
    assert command == (0.25, 0.0, 0.0)


def test_autonomous_mode_ignores_manual_and_selects_nav2():
    owner, command = choose_command(
        now=10.0,
        timeout=0.5,
        mode='AUTONOMOUS',
        sources={
            'WEB_MANUAL': ((0.25, 0.0, 0.0), 10.0),
            'DIRECT_MANUAL': ((0.25, 0.0, 0.0), 10.0),
            'NAV2': ((0.1, 0.0, 0.0), 9.9),
        },
    )
    assert owner == 'NAV2'
    assert command == (0.1, 0.0, 0.0)


def test_tag_route_approach_has_explicit_autonomous_ownership():
    owner, command = choose_command(
        now=10.0,
        timeout=0.5,
        mode='AUTONOMOUS',
        tag_route_state='APPROACH_TAG',
        sources={
            'NAV2': ((0.2, 0.0, 0.0), 10.0),
            'TAG_ROUTE': ((0.0, 0.05, 0.0), 10.0),
        },
    )
    assert owner == 'TAG_ROUTE'
    assert command == (0.0, 0.05, 0.0)


def test_missing_or_stale_active_owner_stops_instead_of_falling_through():
    owner, command = choose_command(
        now=10.6,
        timeout=0.5,
        mode='AUTONOMOUS',
        tag_route_state='APPROACH_TAG',
        sources={
            'NAV2': ((0.2, 0.0, 0.0), 10.5),
            'TAG_ROUTE': ((0.0, 0.05, 0.0), 10.0),
        },
    )
    assert owner == 'NONE'
    assert command == (0.0, 0.0, 0.0)


def test_estop_overrides_fresh_commands_from_every_source():
    owner, command = choose_command(
        now=10.0,
        timeout=0.5,
        mode='MANUAL',
        emergency_stop=True,
        sources={'WEB_MANUAL': ((0.25, 0.0, 0.0), 10.0)},
    )
    assert owner == 'ESTOP'
    assert command == (0.0, 0.0, 0.0)
