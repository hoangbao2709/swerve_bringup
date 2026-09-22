#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"

ok() { printf '[OK] %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1"; }
failures=0

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

if [[ -f "$STACK_RUNTIME_DIR/stack.env" ]]; then
  # shellcheck disable=SC1091
  source "$STACK_RUNTIME_DIR/stack.env"
else
  MODE='unknown'
  BACKEND_PORT="${BACKEND_PORT:-8000}"
  FRONTEND_PORT="${FRONTEND_PORT:-5173}"
  BACKEND_URL="http://127.0.0.1:$BACKEND_PORT"
  FRONTEND_URL="http://127.0.0.1:$FRONTEND_PORT"
  ROS_WS_URL="ws://127.0.0.1:$BACKEND_PORT/ws/ros"
  ROS_DOMAIN_ID=0
fi

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
    if health_flag "$health_key"; then ok "$health_label health"; else warn "$health_label health is not OK"; fi
  done
else
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
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  if ! stack_valid_ros_domain "$ROS_DOMAIN_ID"; then
    warn "invalid ROS_DOMAIN_ID in stack runtime: $ROS_DOMAIN_ID"
    failures=$((failures + 1))
    ROS_DOMAIN_ID=0
  fi
  export ROS_DOMAIN_ID
  set +u
  source "$ROOT_DIR/scripts/ros_env.sh" >/dev/null 2>&1 || true
  set -u
  ROS_NODES="$(timeout 15 ros2 node list 2>/dev/null || true)"
  ROS_CONTROLLERS="$(timeout 20 ros2 control list_controllers --controller-manager /controller_manager --spin-time 2 2>/dev/null || true)"
  ROS_TOPICS="$(timeout 15 ros2 topic list 2>/dev/null || true)"
  topic_present() { printf '%s\n' "$ROS_TOPICS" | rg -q "^${1}$"; }
  if printf '%s\n' "$ROS_NODES" | rg -q '(^|/)swerve_bridge$'; then ok 'ROS bridge node'; else warn 'ROS bridge node not visible'; failures=$((failures + 1)); fi
  if printf '%s\n' "$ROS_NODES" | rg -q '(^|/)controller_manager$'; then ok 'controller_manager'; else warn 'controller_manager not visible'; failures=$((failures + 1)); fi
  for controller in joint_state_broadcaster steering_controller drive_controller; do
    if stack_controller_active "$ROS_CONTROLLERS" "$controller"; then ok "$controller active"; else warn "$controller not active"; failures=$((failures + 1)); fi
  done
  if topic_present /odom; then ok '/odom'; else warn '/odom unavailable'; failures=$((failures + 1)); fi
  if topic_present /odometry/filtered; then ok '/odometry/filtered'; else warn '/odometry/filtered unavailable'; failures=$((failures + 1)); fi
  if topic_present /scan; then ok '/scan'; else warn '/scan unavailable'; failures=$((failures + 1)); fi
  if [[ "${MODE:-}" == mapping ]]; then
    if printf '%s\n' "$ROS_NODES" | rg -q '(^|/)slam_toolbox$'; then ok 'SLAM Toolbox'; else warn 'SLAM Toolbox not visible'; failures=$((failures + 1)); fi
    if topic_present /map; then ok '/map'; else warn '/map unavailable'; failures=$((failures + 1)); fi
  elif [[ "${MODE:-}" == navigation ]]; then
    if printf '%s\n' "$ROS_NODES" | rg -q '(^|/)map_server$'; then ok 'Nav2 map_server'; else warn 'Nav2 map_server not visible'; failures=$((failures + 1)); fi
  fi
else
  warn 'ROS environment unavailable'
  failures=$((failures + 1))
fi

printf 'Status summary: %d failed check(s), mode=%s\n' "$failures" "${MODE:-unknown}"
if (( failures == 0 )); then exit 0; fi
exit 1
