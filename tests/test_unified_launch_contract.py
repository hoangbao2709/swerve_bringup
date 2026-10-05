import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_production_startup_defaults_to_unified_and_accepts_gui_flags_first():
    source = (ROOT / 'scripts/start_stack.sh').read_text(encoding='utf-8')
    assert 'MODE=unified' in source
    assert 'MODE="$1"' in source
    assert '[[ "$1" != -* ]]' in source
    assert 'MODE=$MODE' in source
    assert '--gui) GUI_ARG=true' in source
    assert '--rviz) RVIZ_ARG=true' in source


def test_unified_launch_starts_slam_and_nav2_without_legacy_v30e_localizer():
    source = (ROOT / 'launch/system.launch.py').read_text(encoding='utf-8')
    assert "('mapping', 'unified')" in source
    assert "('navigation', 'unified')" in source
    assert 'condition=legacy_navigation_mode' in source
    assert "'LIVE_SLAM' if '" in source
    assert "'navigation'" in source[source.index('simulated_navigation_mode ='):source.index('require_canonical_map =')]
    assert 'SLAM Toolbox + Nav2 share the live SLAM map' in source


def test_live_slam_nav2_has_no_map_server_and_costmap_subscribes_to_full_map():
    launch = (ROOT / 'swerve_navigation/launch/navigation.launch.py').read_text(encoding='utf-8')
    assert "map_source == 'STATIC_MAP'" in launch
    assert "map_source not in ('LIVE_SLAM', 'STATIC_MAP')" in launch
    assert "'map_server', 'controller_server'" in launch
    assert "'controller_server', 'planner_server'" in launch

    config = yaml.safe_load((ROOT / 'swerve_navigation/config/nav2_params.yaml').read_text(encoding='utf-8'))
    static_layer = config['global_costmap']['global_costmap']['ros__parameters']['static_layer']
    assert static_layer['map_topic'] == '/map'
    assert static_layer['map_subscribe_transient_local'] is True
    assert static_layer['subscribe_to_updates'] is False
    assert config['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False


def test_nav2_progress_checker_counts_terminal_yaw_without_weakening_stuck_limits():
    config = yaml.safe_load((ROOT / 'swerve_navigation/config/nav2_params.yaml').read_text(encoding='utf-8'))
    params = config['controller_server']['ros__parameters']
    checker = params['progress_checker']

    assert checker['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert checker['required_movement_radius'] == 0.30
    assert checker['required_movement_angle'] == 0.20
    assert checker['movement_time_allowance'] == 10.0

    follow_path = params['FollowPath']
    assert follow_path['Oscillation.oscillation_reset_angle'] == follow_path['xy_goal_tolerance']
    assert follow_path['critics'].count('Oscillation') == 1
    bt_params = config['bt_navigator']['ros__parameters']
    assert bt_params['default_server_timeout'] == 500


def test_unified_nav2_readiness_order_matches_lifecycle_manager_start_order():
    readiness = (ROOT / 'scripts/navigation_readiness.py').read_text(encoding='utf-8')
    launch = (ROOT / 'swerve_navigation/launch/navigation.launch.py').read_text(encoding='utf-8')
    expected_order = (
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower',
    )
    readiness_tree = ast.parse(readiness)
    readiness_assignment = next(
        node for node in readiness_tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name)
                and target.id == 'NAV2_LIVE_SLAM_LIFECYCLE_NODES'
                for target in node.targets)
    )
    readiness_order = ast.literal_eval(readiness_assignment.value)
    assert readiness_order == expected_order
    launch_tree = ast.parse(launch)
    configured_order = None
    for node in ast.walk(launch_tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if not (isinstance(key, ast.Constant) and key.value == 'node_names'):
                continue
            if isinstance(value, ast.IfExp) and isinstance(value.orelse, ast.List):
                configured_order = tuple(ast.literal_eval(value.orelse))
                break
    assert configured_order == expected_order


def test_installed_humble_static_layer_exposes_configured_live_map_parameters():
    header = Path('/opt/ros/humble/include/nav2_costmap_2d/static_layer.hpp')
    if not header.is_file():
        return
    source = header.read_text(encoding='utf-8')
    for name in ('map_topic_', 'map_subscribe_transient_local_', 'subscribe_to_updates_'):
        assert name in source
