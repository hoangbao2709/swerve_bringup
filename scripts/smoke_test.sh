#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"

passed=0
failed=0
SHELL_ROS_DOMAIN_ID="${ROS_DOMAIN_ID-}"
check() {
  local name="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    printf 'PASS: %s\n' "$name"
    passed=$((passed + 1))
  else
    printf 'FAIL: %s\n' "$name"
    failed=$((failed + 1))
  fi
}

check_controllers() {
  local controllers
  controllers="$(timeout 20 ros2 control list_controllers --controller-manager /controller_manager --spin-time 2 2>/dev/null)" || return 1
  for name in joint_state_broadcaster steering_controller drive_controller; do
    stack_controller_active "$controllers" "$name" || return 1
  done
}

backend_simulation_time() {
  local payload value
  payload="$(curl -fsS --max-time 3 "$BACKEND_URL/api/health/")" || return 1
  value="$(HEALTH_RESPONSE="$payload" python3 - <<'PY'
import json
import math
import os

payload = json.loads(os.environ['HEALTH_RESPONSE'])
value = payload.get('components', {}).get('diagnostics', {}).get('simulation_time')
if value is None:
    raise SystemExit(1)
value = float(value)
if not math.isfinite(value) or value < 0.0:
    raise SystemExit(1)
print(value)
PY
)" || return 1
  [[ -n "$value" ]]
}

backend_simulation_time_advances() {
  local first second
  first="$(curl -fsS --max-time 3 "$BACKEND_URL/api/health/" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("components", {}).get("diagnostics", {}).get("simulation_time"))')" || return 1
  [[ "$first" != None && "$first" != null ]] || return 1
  # A slow Gazebo RTF can advance less than one simulation tick during a one
  # second wall sleep. Poll for a finite wall deadline and compare the actual
  # bridge value; never substitute wall time for simulation time.
  for _ in $(seq 1 40); do
    sleep 0.5
    second="$(curl -fsS --max-time 3 "$BACKEND_URL/api/health/" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("components", {}).get("diagnostics", {}).get("simulation_time"))')" || continue
    if python3 - "$first" "$second" <<'PY'
import math
import sys

try:
    first, second = map(float, sys.argv[1:])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(first) and math.isfinite(second) and second > first else 1)
PY
    then
      return 0
    fi
  done
  return 1
}

clock_publisher_qos() {
  local deadline info
  deadline=$((SECONDS + 40))
  while (( SECONDS < deadline )); do
    info="$(timeout 15 ros2 topic info --no-daemon --spin-time 3 /clock --verbose 2>/dev/null || true)"
    if QOS_INFO="$info" python3 - <<'PY'
import os
import re
import sys

text = os.environ.get('QOS_INFO', '')
blocks = re.split(r'(?=Endpoint type:)', text)
publishers = [block for block in blocks if 'Endpoint type: PUBLISHER' in block]
subscriptions = [block for block in blocks if 'Endpoint type: SUBSCRIPTION' in block]

def matches(block):
    reliability = re.search(r'Reliability:\s*(\S+)', block, re.IGNORECASE)
    durability = re.search(r'Durability:\s*(\S+)', block, re.IGNORECASE)
    return (reliability and reliability.group(1).upper() == 'BEST_EFFORT'
            and durability and durability.group(1).upper() == 'VOLATILE')

# Verify both sides of the live /clock connection. The subscriber check is
# narrowed to swerve_bridge so this cannot pass from an unrelated RViz client.
bridge_subscriptions = [
    block for block in subscriptions
    if re.search(r'Node name:\s*swerve_bridge\b', block)
]
if not any(matches(block) for block in publishers):
    raise SystemExit('no BEST_EFFORT/VOLATILE /clock publisher')
if not any(matches(block) for block in bridge_subscriptions):
    raise SystemExit('swerve_bridge /clock subscription is not BEST_EFFORT/VOLATILE')
PY
    then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

ROS_DATA_PROBE_OUTPUT=''
ros_data_probe_once() {
  if [[ -z "$ROS_DATA_PROBE_OUTPUT" ]]; then
    data_topics=(/clock /joint_states /odom /odometry/filtered /lidar/points /lidar/points_filtered /scan)
    if [[ "${MODE:-}" == mapping || "${MODE:-}" == navigation ]]; then
      data_topics+=(/map)
    fi
    ROS_DATA_PROBE_OUTPUT="$(stack_ros_data_probe 20 "${data_topics[@]}" 2>&1 || true)"
  fi
}

ros_data_topic() {
  ros_data_probe_once
  printf '%s\n' "$ROS_DATA_PROBE_OUTPUT" | rg -Fxq "OK: $1 message"
}

if [[ -f "$ROOT_DIR/waretwin/backend/.env" ]]; then
  # Load the local bridge token for the authentication probe, then let the
  # selected fallback ports below override the .env defaults.
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/waretwin/backend/.env"
  set +a
fi
if [[ -f "$STACK_RUNTIME_DIR/stack.env" ]] && stack_runtime_active; then
  # shellcheck disable=SC1091
  source "$STACK_RUNTIME_DIR/stack.env"
elif [[ -f "$STACK_RUNTIME_DIR/stack.env" ]]; then
  printf 'INFO: stale runtime stack.env ignored; using backend/.env\n'
fi
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if ! stack_valid_ros_domain "$ROS_DOMAIN_ID"; then
  printf 'FAIL: valid stack ROS_DOMAIN_ID\n'
  failed=$((failed + 1))
  ROS_DOMAIN_ID=0
fi
export ROS_DOMAIN_ID
printf 'ROS domain: %s (runtime preferred; shell=%s)\n' "$ROS_DOMAIN_ID" "${SHELL_ROS_DOMAIN_ID:-<unset>}"
# Resolve URLs only after both environment files have been loaded.  This keeps
# direct `run.sh` deployments and fallback-port stack deployments consistent.
BACKEND_URL="${BACKEND_URL:-http://127.0.0.1:${BACKEND_PORT:-8000}}"
FRONTEND_URL="${FRONTEND_URL:-http://127.0.0.1:${FRONTEND_PORT:-5173}}"
ROS_WS_URL="${ROS_WS_URL:-ws://127.0.0.1:${BACKEND_PORT:-8000}/ws/ros}"

check 'backend health' curl -fsS --max-time 3 "$BACKEND_URL/api/health/"
check 'frontend HTTP' curl -fsS --max-time 3 "$FRONTEND_URL/"
check 'backend simulation_time is non-null and non-negative' backend_simulation_time
check 'backend simulation_time advances from /clock' backend_simulation_time_advances
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
  printf 'FAIL: backend WebSocket bridge authentication\n'
  failed=$((failed + 1))
fi

if [[ -f "$ROOT_DIR/install/local_setup.bash" ]]; then
  set +u
  source "$ROOT_DIR/scripts/ros_env.sh" >/dev/null 2>&1 || true
  set -u
  export ROS_DOMAIN_ID
  check 'ROS bridge node' timeout 12 bash -c "ros2 node list --no-daemon --spin-time 5 | rg -q '/swerve_bridge$|^/swerve_bridge$'"
  check 'controller_manager' timeout 12 bash -c 'ros2 node list --no-daemon --spin-time 5 | rg -q "/controller_manager$|^/controller_manager$"'
  check 'controllers active' check_controllers
  check 'robot spawned in Gazebo' timeout 8 bash -c 'gz model -m swerve_base -i | rg -q swerve_base'
  check '/clock data' ros_data_topic /clock
  check '/clock publisher QoS BEST_EFFORT/VOLATILE' clock_publisher_qos
  check '/joint_states data' ros_data_topic /joint_states
  check '/odom data' ros_data_topic /odom
  check '/odometry/filtered data' ros_data_topic /odometry/filtered
  check '/lidar/points data' ros_data_topic /lidar/points
  check '/lidar/points_filtered data' ros_data_topic /lidar/points_filtered
  check '/scan data' ros_data_topic /scan
  tf_frames=(odom base_link map base_link)
  if [[ "${MODE:-mapping}" == mapping || "${MODE:-mapping}" == navigation ]]; then
    tf_frames+=(map odom)
  fi
  tf_output="$(stack_ros_tf_probe 20 "${tf_frames[@]}" 2>&1 || true)"
  if printf '%s\n' "$tf_output" | rg -Fxq 'OK: odom -> base_link'; then
    check 'TF odom -> base_link' true
  else
    check 'TF odom -> base_link' false
  fi
  if printf '%s\n' "$tf_output" | rg -Fxq 'OK: map -> base_link'; then
    check 'TF map -> base_link' true
  else
    check 'TF map -> base_link' false
  fi
  if [[ "${MODE:-mapping}" == mapping ]]; then
    check 'SLAM Toolbox' timeout 12 bash -c 'ros2 node list --no-daemon --spin-time 5 | rg -q "/slam_toolbox$|^/slam_toolbox$"'
    check '/map data (mapping)' ros_data_topic /map
    if printf '%s\n' "$tf_output" | rg -Fxq 'OK: map -> odom'; then
      check 'TF map -> odom (mapping)' true
    else
      check 'TF map -> odom (mapping)' false
    fi
  fi
  if [[ "${MODE:-mapping}" == navigation ]]; then
    nav_nodes=(map_server planner_server controller_server behavior_server bt_navigator waypoint_follower)
    lifecycle_output="$(stack_ros_lifecycle_probe 25 "${nav_nodes[@]}" 2>&1 || true)"
    for nav_node in "${nav_nodes[@]}"; do
      check "Nav2 $nav_node" timeout 12 bash -c "ros2 node list --no-daemon --spin-time 5 | rg -q '(^|/)$nav_node$'"
      if printf '%s\n' "$lifecycle_output" | rg -Fxq "OK: /$nav_node active [3]"; then
        printf 'PASS: Nav2 %s lifecycle active\n' "$nav_node"
        passed=$((passed + 1))
      else
        printf 'FAIL: Nav2 %s lifecycle active\n' "$nav_node"
        failed=$((failed + 1))
      fi
    done
    check '/map data (navigation)' ros_data_topic /map
    if printf '%s\n' "$tf_output" | rg -Fxq 'OK: map -> odom'; then
      check 'TF map -> odom (navigation)' true
    else
      check 'TF map -> odom (navigation)' false
    fi
    if [[ -n "${MAP_FILE:-}" ]]; then
      expected_map="$(readlink -f -- "$MAP_FILE")"
      check "map_server yaml_filename=$expected_map" bash -c "timeout 6 ros2 param get /map_server yaml_filename | rg -Fq '$expected_map'"
    fi
  fi
else
  printf 'FAIL: ROS workspace\n'
  failed=$((failed + 1))
fi

printf '\nTOTAL=%d PASSED=%d FAILED=%d\n' "$((passed + failed))" "$passed" "$failed"
((failed == 0))
