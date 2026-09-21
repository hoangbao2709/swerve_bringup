#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"

passed=0
failed=0
check() {
  local name="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    printf '[PASS] %s\n' "$name"
    passed=$((passed + 1))
  else
    printf '[FAIL] %s\n' "$name"
    failed=$((failed + 1))
  fi
}

if [[ -f "$ROOT_DIR/waretwin/backend/.env" ]]; then
  # Load the local bridge token for the authentication probe, then let the
  # selected fallback ports below override the .env defaults.
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/waretwin/backend/.env"
  set +a
fi
if [[ -f "$STACK_RUNTIME_DIR/stack.env" ]]; then
  # shellcheck disable=SC1091
  source "$STACK_RUNTIME_DIR/stack.env"
fi
# Resolve URLs only after both environment files have been loaded.  This keeps
# direct `run.sh` deployments and fallback-port stack deployments consistent.
BACKEND_URL="${BACKEND_URL:-http://127.0.0.1:${BACKEND_PORT:-8000}}"
FRONTEND_URL="${FRONTEND_URL:-http://127.0.0.1:${FRONTEND_PORT:-5173}}"
ROS_WS_URL="${ROS_WS_URL:-ws://127.0.0.1:${BACKEND_PORT:-8000}/ws/ros}"

check 'backend health' curl -fsS --max-time 3 "$BACKEND_URL/api/health/"
check 'frontend HTTP' curl -fsS --max-time 3 "$FRONTEND_URL/"
PYTHON_BIN="$ROOT_DIR/waretwin/backend/.venv/bin/python"
[[ -x "$PYTHON_BIN" ]] || PYTHON_BIN="$(command -v python3)"
check 'database/Django' bash -c "cd '$ROOT_DIR/waretwin/backend' && '$PYTHON_BIN' manage.py check"

if command -v python3 >/dev/null 2>&1; then
  check 'backend WebSocket bridge authentication' env WARETWIN_ROS_BRIDGE_TOKEN="${WARETWIN_ROS_BRIDGE_TOKEN:-}" ROS_WS_URL="$ROS_WS_URL" "$PYTHON_BIN" - <<'PY'
import os
import urllib.parse
import websocket
token = os.environ.get('WARETWIN_ROS_BRIDGE_TOKEN', '')
url = os.environ['ROS_WS_URL'] + '?' + urllib.parse.urlencode({'token': token})
ws = websocket.create_connection(url, timeout=3)
ws.close()
PY
else
  printf '[FAIL] backend WebSocket bridge authentication\n'
  failed=$((failed + 1))
fi

if [[ -f "$ROOT_DIR/install/local_setup.bash" ]]; then
  set +u
  source "$ROOT_DIR/scripts/ros_env.sh" >/dev/null 2>&1 || true
  set -u
  check 'ROS bridge node' timeout 5 bash -c "ros2 node list | rg -q '/swerve_bridge$|^/swerve_bridge$'"
  check 'controller_manager' timeout 5 bash -c 'ros2 node list | rg -q "/controller_manager$|^/controller_manager$"'
  check '/odom or /odometry/filtered' timeout 5 bash -c 'ros2 topic info /odometry/filtered >/dev/null || ros2 topic info /odom >/dev/null'
  check '/scan' timeout 5 ros2 topic info /scan
  if [[ "${MODE:-mapping}" == mapping ]]; then check '/map (mapping)' timeout 5 ros2 topic info /map; fi
  if [[ "${MODE:-mapping}" == navigation ]]; then check 'Nav2 map_server' timeout 5 bash -c 'ros2 node list | rg -q "/map_server$|^/map_server$"'; fi
else
  printf '[FAIL] ROS workspace\n'
  failed=$((failed + 1))
fi

printf '\nTOTAL=%d PASSED=%d FAILED=%d\n' "$((passed + failed))" "$passed" "$failed"
((failed == 0))
