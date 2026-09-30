"""Regression coverage for lost DDS replies and controller dependency safety."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location(
    'controller_spawner', Path(__file__).resolve().parents[1] / 'scripts/controller_spawner.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def row(name, state='active', claims=None):
    return SimpleNamespace(name=name, state=state, claimed_interfaces=list(
        module.CLAIMS[name] if claims is None else claims))


def caller():
    node = Mock()
    node.response_timeout = 0.01
    node.manager_deadline = float('inf')
    client = Mock()
    client.service_is_ready.return_value = True
    node.service_clients = {'list_controllers': client, 'load_controller': client}
    return node, client


def test_lost_read_response_retries_on_same_client_without_pending_backlog():
    node, client = caller()
    lost, received = Mock(), Mock()
    lost.done.return_value = False
    received.done.return_value = True
    received.cancelled.return_value = False
    received.exception.return_value = None
    received.result.return_value = 'controller rows'
    client.call_async.side_effect = [lost, received]
    with patch.object(module.rclpy, 'spin_until_future_complete'):
        result = module.ControllerSpawner.call(node, 'list_controllers', object(), read_only=True)
    assert result == 'controller rows'
    assert client.call_async.call_count == 2
    client.remove_pending_request.assert_called_once_with(lost)
    lost.cancel.assert_called_once()


def test_mutation_with_lost_reply_is_never_replayed():
    node, client = caller()
    client.call_async.return_value.done.return_value = False
    with patch.object(module.rclpy, 'spin_until_future_complete'):
        assert module.ControllerSpawner.call(node, 'load_controller', object()) is None
    assert client.call_async.call_count == 1


def test_lost_mutation_ack_reconciles_actual_state_without_repeating_mutation():
    node = Mock(controller='drive_controller', response_timeout=1.0)
    node.call.return_value = None
    rows = [row('drive_controller', 'unconfigured')]
    node.rows.return_value = rows
    node.state.return_value = 'unconfigured'
    assert module.ControllerSpawner.mutate(node, 'load_controller', object(), 'unconfigured') == rows
    node.call.assert_called_once()


def test_active_confirmation_requires_predecessors_and_unique_expected_claims():
    import pytest
    rows = [row(name) for name in module.ORDER]
    module.validate_active('drive_controller', rows)
    rows[1].state = 'inactive'
    with pytest.raises(RuntimeError, match='predecessor'):
        module.validate_active('drive_controller', rows)
    rows[1].state = 'active'
    rows[2].claimed_interfaces = []
    with pytest.raises(RuntimeError, match='claimed'):
        module.validate_active('drive_controller', rows)
    rows[2] = row('drive_controller')
    rows.append(SimpleNamespace(name='duplicate', state='active', claimed_interfaces=rows[2].claimed_interfaces))
    with pytest.raises(RuntimeError, match='duplicate'):
        module.validate_active('drive_controller', rows)


def test_exhausted_read_responses_fail_instead_of_waiting_forever():
    import pytest
    node, client = caller()
    client.call_async.return_value.done.return_value = False
    with patch.object(module.rclpy, 'spin_until_future_complete'):
        with pytest.raises(RuntimeError, match='3 bounded read-only attempts'):
            module.ControllerSpawner.call(node, 'list_controllers', object(), read_only=True)
    assert client.call_async.call_count == 3


def test_spawner_constructs_real_humble_node_and_persistent_service_clients():
    module.rclpy.init()
    node = None
    try:
        node = module.ControllerSpawner('joint_state_broadcaster')
        assert set(node.service_clients) == {
            'list_controllers', 'load_controller', 'configure_controller', 'switch_controller'}
        assert len(tuple(node.clients)) == 4
    finally:
        if node is not None:
            node.destroy_node()
        module.rclpy.shutdown()
