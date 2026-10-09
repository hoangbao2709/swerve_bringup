#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"
if python3 "$ROOT_DIR/waretwin/runtime/control.py" --managed; then
  python3 "$ROOT_DIR/waretwin/runtime/control.py" "$@"
  exit $?
fi

ok() { printf '[OK] %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1"; }
failures=0
SHELL_ROS_DOMAIN_ID="${ROS_DOMAIN_ID-}"
SHELL_RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION-}"
SHELL_ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY-}"
RUNTIME_DOMAIN_SOURCE='default'
RUNTIME_STACK_ACTIVE=0

BACKEND_PORT_OVERRIDE=''
FRONTEND_PORT_OVERRIDE=''
while (($#)); do
  case "$1" in
    --backend-port)
      shift
      [[ $# -gt 0 ]] || { echo '--backend-port needs a value' >&2; exit 2; }
      BACKEND_PORT_OVERRIDE="$1"
      ;;
    --frontend-port)
      shift
      [[ $# -gt 0 ]] || { echo '--frontend-port needs a value' >&2; exit 2; }
      FRONTEND_PORT_OVERRIDE="$1"
      ;;
    -h|--help)
      sed -n '1,90p' "$ROOT_DIR/scripts/status_stack.sh"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

if [[ -f "$STACK_RUNTIME_DIR/stack.env" ]] && stack_runtime_active; then
  # shellcheck disable=SC1091
  source "$STACK_RUNTIME_DIR/stack.env"
  RUNTIME_STACK_ACTIVE=1
  RUNTIME_DOMAIN_SOURCE='runtime stack.env'
else
  if [[ -f "$STACK_RUNTIME_DIR/stack.env" ]]; then
    warn 'runtime stack.env is stale; falling back to backend/.env'
  fi
  MODE='unknown'
  GAZEBO_GUI=false
  RVIZ=false
  BACKEND_PORT="${BACKEND_PORT:-8000}"
  FRONTEND_PORT="${FRONTEND_PORT:-5173}"
  BACKEND_URL="http://127.0.0.1:$BACKEND_PORT"
  FRONTEND_URL="http://127.0.0.1:$FRONTEND_PORT"
  ROS_WS_URL="ws://127.0.0.1:$BACKEND_PORT/ws/ros"
  if [[ -f "$ROOT_DIR/waretwin/backend/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "$ROOT_DIR/waretwin/backend/.env"
    set +a
    RUNTIME_DOMAIN_SOURCE='backend/.env'
  fi
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
fi

GAZEBO_GUI="${GAZEBO_GUI:-false}"
RVIZ="${RVIZ:-false}"
ROS_BRIDGE_STATUS='UNKNOWN'

RUNTIME_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "$RUNTIME_STACK_ACTIVE" -eq 0 ]]; then
  BACKEND_PORT="${BACKEND_PORT:-8000}"
  FRONTEND_PORT="${FRONTEND_PORT:-5173}"
  BACKEND_URL="http://127.0.0.1:$BACKEND_PORT"
  FRONTEND_URL="http://127.0.0.1:$FRONTEND_PORT"
  ROS_WS_URL="ws://127.0.0.1:$BACKEND_PORT/ws/ros"
fi
printf 'ROS domain: %s (%s)\n' "$RUNTIME_ROS_DOMAIN_ID" "$RUNTIME_DOMAIN_SOURCE"

if [[ -n "$BACKEND_PORT_OVERRIDE" ]]; then
  BACKEND_PORT="$BACKEND_PORT_OVERRIDE"
  BACKEND_URL="http://127.0.0.1:$BACKEND_PORT"
  ROS_WS_URL="ws://127.0.0.1:$BACKEND_PORT/ws/ros"
fi
if [[ -n "$FRONTEND_PORT_OVERRIDE" ]]; then
  FRONTEND_PORT="$FRONTEND_PORT_OVERRIDE"
  FRONTEND_URL="http://127.0.0.1:$FRONTEND_PORT"
fi

for component in backend frontend ros; do
  if stack_owned_pid "$component" || stack_owned_group "$component"; then
    ok "$component process (PID $(stack_pid "$component"))"
  else
    warn "$component process is not running under this stack"
    failures=$((failures + 1))
  fi
done

if response="$(curl -fsS --max-time 3 "$BACKEND_URL/api/health/" 2>/dev/null)"; then
  API_HEALTH_AVAILABLE=1
  ok "backend HTTP $BACKEND_URL/api/health/"
  printf '%s\n' "$response"
  health_flag() {
    local key="$1"
    HEALTH_RESPONSE="$response" python3 - "$key" <<'PY'
import json
import os
import sys

try:
    payload = json.loads(os.environ.get('HEALTH_RESPONSE', '{}'))
    value = payload.get(sys.argv[1])
except (TypeError, ValueError, IndexError):
    value = False
raise SystemExit(0 if value is True else 1)
PY
  }
  for health_item in 'database Database' 'websocket WebSocket' 'ros_bridge ROS bridge' 'ros ROS graph' 'gazebo Gazebo'; do
    health_key="${health_item%% *}"
    health_label="${health_item#* }"
    if health_flag "$health_key"; then
      ok "$health_label health"
    else
      warn "$health_label health is not OK"
      failures=$((failures + 1))
    fi
  done
else
  API_HEALTH_AVAILABLE=0
  warn "backend health unavailable at $BACKEND_URL/api/health/"
  failures=$((failures + 1))
fi

if curl -fsS --max-time 3 "$FRONTEND_URL/" >/dev/null 2>&1; then
  ok "frontend HTTP $FRONTEND_URL"
else
  warn "frontend HTTP unavailable at $FRONTEND_URL"
  failures=$((failures + 1))
fi

if command -v ros2 >/dev/null 2>&1 && [[ -f "$ROOT_DIR/install/local_setup.bash" ]]; then
  ROS_DOMAIN_ID="$RUNTIME_ROS_DOMAIN_ID"
  if ! stack_valid_ros_domain "$ROS_DOMAIN_ID"; then
    warn "invalid ROS domain in runtime state: $ROS_DOMAIN_ID"
    failures=$((failures + 1))
    ROS_DOMAIN_ID=0
  fi
  export ROS_DOMAIN_ID
  set +u
  if ! source "$ROOT_DIR/scripts/ros_env.sh" >/dev/null 2>&1; then
    warn 'canonical ROS runtime environment failed to load'
    exit 1
  fi
  set -u
  # ros_env.sh may read backend/.env for manual use; the explicit export above
  # keeps this status probe on the runtime domain selected by start_stack.
  export ROS_DOMAIN_ID="$RUNTIME_ROS_DOMAIN_ID"
  printf 'RMW implementation: %s\n' "${RMW_IMPLEMENTATION:-<unset>}"
  printf 'ROS_LOCALHOST_ONLY: %s\n' "${ROS_LOCALHOST_ONLY:-<unset>}"
  printf 'Fast DDS transports: %s\n' "${FASTDDS_BUILTIN_TRANSPORTS:-<unset>}"
  ROS_NODES="$(timeout 15 ros2 node list --no-daemon --spin-time 5 2>/dev/null || true)"
  ROS_CONTROLLERS="$(timeout 20 ros2 control list_controllers --controller-manager /controller_manager --spin-time 2 2>/dev/null || true)"
  ROS_TOPICS="$(timeout 15 ros2 topic list --no-daemon --spin-time 2 2>/dev/null || true)"
  node_present() { printf '%s\n' "$ROS_NODES" | rg -q "(^|/)${1}$"; }
  topic_present() { printf '%s\n' "$ROS_TOPICS" | rg -q "^${1}$"; }
  ROS_DATA_PROBE_OUTPUT=''
  ROS_DATA_PROBE_STATUS=1
  ros_data_probe_once() {
    if [[ -z "$ROS_DATA_PROBE_OUTPUT" ]]; then
      data_topics=(/joint_states /odom /odometry/filtered /lidar/points /lidar/points_filtered /scan /clock)
      if [[ "${MODE:-}" == mapping || "${MODE:-}" == navigation ]]; then
        data_topics+=(/map)
      fi
      if ROS_DATA_PROBE_OUTPUT="$(stack_ros_data_probe 20 "${data_topics[@]}" 2>&1)"; then
        ROS_DATA_PROBE_STATUS=0
      else
        ROS_DATA_PROBE_STATUS=$?
      fi
    fi
  }
  topic_data() {
    ros_data_probe_once
    printf '%s\n' "$ROS_DATA_PROBE_OUTPUT" | rg -Fxq "OK: $1 message"
  }

  cli_bridge=0; cli_controller_manager=0; cli_controllers=0; cli_slam=0; cli_lidar=0; cli_tf=0
  if node_present swerve_bridge; then
    ok 'ROS bridge node'
    cli_bridge=1
    ROS_BRIDGE_STATUS='RUNNING'
  else
    warn 'ROS bridge node not visible'
    ROS_BRIDGE_STATUS='STOPPED'
    failures=$((failures + 1))
  fi
  if node_present controller_manager; then ok 'controller_manager'; cli_controller_manager=1; else warn 'controller_manager not visible'; failures=$((failures + 1)); fi
  cli_controllers=1
  for controller in joint_state_broadcaster steering_controller drive_controller; do
    if stack_controller_active "$ROS_CONTROLLERS" "$controller"; then ok "$controller active"; else warn "$controller not active"; cli_controllers=0; failures=$((failures + 1)); fi
  done

  for topic in /joint_states /odom /odometry/filtered /lidar/points /lidar/points_filtered /scan /clock; do
    if topic_data "$topic"; then
      ok "$topic publishing"
      [[ "$topic" == /lidar/points ]] && cli_lidar=1
    else
      warn "$topic exists but no data received"
      failures=$((failures + 1))
    fi
  done
  if stack_ros_tf_resolves odom base_link 20; then ok 'TF odom -> base_link'; cli_tf=1; else warn 'TF odom -> base_link unavailable'; failures=$((failures + 1)); fi
  if stack_ros_tf_resolves map base_link 20; then ok 'TF map -> base_link'; else warn 'TF map -> base_link unavailable'; failures=$((failures + 1)); fi

  if [[ "${MODE:-}" == mapping ]]; then
    if node_present slam_toolbox; then ok 'SLAM Toolbox'; cli_slam=1; else warn 'SLAM Toolbox not visible'; failures=$((failures + 1)); fi
    if topic_data /map; then ok '/map publishing'; else warn '/map exists but no data received'; failures=$((failures + 1)); fi
    if stack_ros_tf_resolves map odom 20; then ok 'TF map -> odom'; else warn 'TF map -> odom unavailable'; failures=$((failures + 1)); fi
  elif [[ "${MODE:-}" == navigation ]]; then
    cli_nav_nodes=1
    nav_nodes=(map_server planner_server controller_server behavior_server bt_navigator waypoint_follower)
    nav_lifecycle_output="$(stack_ros_lifecycle_probe 25 "${nav_nodes[@]}" 2>&1 || true)"
    for nav_node in "${nav_nodes[@]}"; do
      if node_present "$nav_node"; then ok "Nav2 $nav_node"; else warn "Nav2 $nav_node not visible"; cli_nav_nodes=0; failures=$((failures + 1)); fi
      if printf '%s\n' "$nav_lifecycle_output" | rg -Fxq "OK: /$nav_node active [3]"; then
        ok "Nav2 $nav_node active"
      else
        warn "Nav2 $nav_node is not active"
        cli_nav_nodes=0
        failures=$((failures + 1))
      fi
    done
    if topic_data /map; then ok '/map publishing'; else warn '/map exists but no data received'; failures=$((failures + 1)); fi
    if stack_ros_tf_resolves map odom 20; then ok 'TF map -> odom'; else warn 'TF map -> odom unavailable'; failures=$((failures + 1)); fi
    if [[ -n "${MAP_FILE:-}" ]]; then
      expected_map="$(readlink -f -- "$MAP_FILE")"
      map_param="$(timeout 20 ros2 param get /map_server yaml_filename 2>/dev/null || true)"
      if printf '%s\n' "$map_param" | rg -Fq "$expected_map"; then ok "map_server yaml_filename=$expected_map"; else warn "map_server yaml_filename does not match $expected_map"; failures=$((failures + 1)); fi
    fi
  fi

  if [[ "$API_HEALTH_AVAILABLE" -eq 1 ]]; then
    api_diag_flag() {
      HEALTH_RESPONSE="$response" python3 - "$1" <<'PY'
import json
import os
import sys

try:
    payload = json.loads(os.environ.get('HEALTH_RESPONSE', '{}'))
    values = payload.get('components', {}).get('diagnostics', {})
    value = values.get(sys.argv[1])
except (TypeError, ValueError, IndexError):
    value = False
raise SystemExit(0 if value is True else 1)
PY
    }
    api_controller=0; api_slam=0; api_lidar=0
    api_diag_flag controller_manager && api_controller=1 || true
    api_diag_flag slam && api_slam=1 || true
    api_diag_flag lidar && api_lidar=1 || true
    disagreement=0
    if (( api_controller != cli_controller_manager || api_controller != cli_controllers )); then disagreement=1; fi
    if [[ "${MODE:-}" == mapping && $api_slam -ne $cli_slam ]]; then disagreement=1; fi
    # The bridge reports lidar health from real callback age; the CLI sample is
    # an independent truth source even when the API currently says healthy.
    if (( api_lidar != cli_lidar )); then disagreement=1; fi
    if (( disagreement )); then
      warn 'ROS CLI/API disagreement'
      printf '  runtime domain: %s\n  shell domain: %s\n  RMW: %s\n  ROS_LOCALHOST_ONLY: %s\n' \
        "$RUNTIME_ROS_DOMAIN_ID" "${SHELL_ROS_DOMAIN_ID:-<unset>}" \
        "${RMW_IMPLEMENTATION:-${SHELL_RMW_IMPLEMENTATION:-<unset>}}" \
        "${ROS_LOCALHOST_ONLY:-${SHELL_ROS_LOCALHOST_ONLY:-<unset>}}"
      failures=$((failures + 1))
    fi
  fi
else
  warn 'ROS environment unavailable'
  failures=$((failures + 1))
fi

if stack_owned_group ros && stack_group_has_process ros gzserver; then
  GAZEBO_SERVER_STATUS='RUNNING'
else
  GAZEBO_SERVER_STATUS='STOPPED'
fi
if stack_owned_group ros && stack_group_has_process ros gzclient; then
  GAZEBO_GUI_STATUS='RUNNING'
elif [[ "${GAZEBO_GUI,,}" == true || "$GAZEBO_GUI" == 1 ]]; then
  GAZEBO_GUI_STATUS='STOPPED'
else
  GAZEBO_GUI_STATUS='DISABLED'
fi
if stack_owned_group ros && stack_group_has_process ros rviz2; then
  RVIZ_STATUS='RUNNING'
elif [[ "${RVIZ,,}" == true || "$RVIZ" == 1 ]]; then
  RVIZ_STATUS='STOPPED'
else
  RVIZ_STATUS='DISABLED'
fi
if stack_owned_pid frontend || stack_owned_group frontend; then FRONTEND_STATUS='RUNNING'; else FRONTEND_STATUS='STOPPED'; fi
if stack_owned_pid backend || stack_owned_group backend; then BACKEND_STATUS='RUNNING'; else BACKEND_STATUS='STOPPED'; fi

printf 'Gazebo server : %s\n' "$GAZEBO_SERVER_STATUS"
printf 'Gazebo GUI    : %s\n' "$GAZEBO_GUI_STATUS"
printf 'RViz          : %s\n' "$RVIZ_STATUS"
printf 'Frontend      : %s\n' "$FRONTEND_STATUS"
printf 'Backend       : %s\n' "$BACKEND_STATUS"
printf 'ROS bridge    : %s\n' "$ROS_BRIDGE_STATUS"

printf 'Status summary: %d failed check(s), mode=%s\n' "$failures" "${MODE:-unknown}"
if (( failures == 0 )); then exit 0; fi
exit 1
