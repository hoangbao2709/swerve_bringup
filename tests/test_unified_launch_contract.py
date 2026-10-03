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


def test_installed_humble_static_layer_exposes_configured_live_map_parameters():
    header = Path('/opt/ros/humble/include/nav2_costmap_2d/static_layer.hpp')
    if not header.is_file():
        return
    source = header.read_text(encoding='utf-8')
    for name in ('map_topic_', 'map_subscribe_transient_local_', 'subscribe_to_updates_'):
        assert name in source
