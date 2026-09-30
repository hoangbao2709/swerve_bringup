import importlib.util
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]


def launch_module():
    path = ROOT / 'launch' / 'gazebo.launch.py'
    spec = importlib.util.spec_from_file_location('gazebo_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_controller_launch_chain_advances_only_on_successful_process_exit():
    from launch.actions import LogInfo, Shutdown

    module = launch_module()
    next_action = LogInfo(msg='start dependent controller')
    handler = module._advance_after_success('joint_state_broadcaster activation', [next_action])

    assert handler(SimpleNamespace(returncode=0), None) == [next_action]

    failed = handler(SimpleNamespace(returncode=1), None)
    assert any(isinstance(action, LogInfo) for action in failed)
    assert any(isinstance(action, Shutdown) for action in failed)


def test_controller_launch_chain_does_not_treat_unknown_exit_as_success():
    from launch.actions import Shutdown

    module = launch_module()
    handler = module._advance_after_success('drive_controller activation', [])

    actions = handler(SimpleNamespace(returncode=None), None)

    assert any(isinstance(action, Shutdown) for action in actions)
