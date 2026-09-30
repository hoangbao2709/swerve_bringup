import importlib.util
import os
from pathlib import Path
import subprocess
import pytest


ROOT = Path(__file__).resolve().parents[1]


def navigation_module():
    spec = importlib.util.spec_from_file_location(
        'navigation_environment_under_test',
        ROOT / 'swerve_navigation/launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_nav2_retains_installed_bond_patch_without_unrelated_overlays(tmp_path, monkeypatch):
    module = navigation_module()
    library = tmp_path / 'lib/libbondcpp.so'
    library.parent.mkdir()
    library.touch()
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: str(tmp_path))
    monkeypatch.setenv('LD_LIBRARY_PATH', '/unrelated/overlay/lib')
    assert module._nav2_environment() == {
        'AMENT_PREFIX_PATH': '/opt/ros/humble',
        'LD_LIBRARY_PATH': f'{tmp_path}/lib:/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu',
    }


def test_nav2_uses_system_libraries_when_no_compat_library_installed(tmp_path, monkeypatch):
    module = navigation_module()
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: str(tmp_path))
    assert module._nav2_environment()['LD_LIBRARY_PATH'] == (
        '/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu')


def test_actual_loader_uses_patch_under_navigation_launch_environment():
    module = navigation_module()
    env = dict(os.environ, **module._nav2_environment())
    prefix = Path(module.get_package_prefix('swerve_bringup'))
    library = prefix / 'lib/libbondcpp.so'
    if not library.is_file():
        pytest.skip('version-gated Humble compatibility library is not installed')
    result = subprocess.run([
        '/usr/bin/python3', '-c',
        "import ctypes; from pathlib import Path; ctypes.CDLL('libbondcpp.so'); "
        "print(Path('/proc/self/maps').read_text())",
    ], env=env, check=True, capture_output=True, text=True)
    assert str(library.resolve()) in result.stdout or str(library) in result.stdout
    assert '/opt/ros/humble/lib/libbondcpp.so' not in result.stdout
