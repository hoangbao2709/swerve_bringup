#!/usr/bin/env bash
# Usage: start_stack.sh [unified|mapping|navigation] [--map PATH] [--gui] [--rviz]
# GUI and RViz default to off; --headless and the --no-* switches remain aliases.
set -euo pipefail

# Share one monotonic startup origin with the readiness probe.  /proc/uptime
# uses the kernel monotonic clock and avoids wall-clock/NTP adjustments.
read -r STACK_START_MONOTONIC_S _ < /proc/uptime
export WARETWIN_STACK_START_MONOTONIC_S="$STACK_START_MONOTONIC_S"
echo "T0_START_STACK=PASS monotonic_s=$STACK_START_MONOTONIC_S"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"
NAV2_LIFECYCLE_STATE_FILE="$STACK_RUNTIME_DIR/nav2-lifecycle-startup.json"
export WARETWIN_NAV2_LIFECYCLE_STATE_FILE="$NAV2_LIFECYCLE_STATE_FILE"

MODE=unified
if (($#)) && [[ "$1" != -* ]]; then
  MODE="$1"
  shift
fi
if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  sed -n '1,125p' "$0"
  exit 0
fi
[[ "$MODE" == unified || "$MODE" == mapping || "$MODE" == navigation ]] || { echo "Usage: $0 [unified|mapping|navigation] [options]" >&2; exit 2; }

GUI_ARG=false
RVIZ_ARG=false
BACKEND_PORT_ARG=""
FRONTEND_PORT_ARG=""
MAP_FILE=""
WORLD_FILE=""
EXPLICIT_MAP=0
EXPLICIT_WORLD=0
ALLOW_DEV_WORLD_SELECTED="${WARETWIN_ALLOW_DEV_WORLD:-${ALLOW_DEV_WORLD:-false}}"
ALLOW_DEV_WORLD_CLI=0
ROBOT_ID="${WARETWIN_ROBOT_ID:-R01}"
NAMESPACE="${WARETWIN_ROS_NAMESPACE:-}"

while (($#)); do
  case "$1" in
    --headless) GUI_ARG=false; RVIZ_ARG=false ;;
    --gui) GUI_ARG=true ;;
    --rviz) RVIZ_ARG=true ;;
    --no-rviz) RVIZ_ARG=false ;;
    --no-gazebo-gui) GUI_ARG=false ;;
    --backend-port) shift; [[ $# -gt 0 ]] || { echo '--backend-port needs a value' >&2; exit 2; }; BACKEND_PORT_ARG="$1" ;;
    --frontend-port) shift; [[ $# -gt 0 ]] || { echo '--frontend-port needs a value' >&2; exit 2; }; FRONTEND_PORT_ARG="$1" ;;
    --map) shift; [[ $# -gt 0 ]] || { echo '--map needs a YAML path' >&2; exit 2; }; MAP_FILE="$(readlink -f -- "$1")"; [[ -f "$MAP_FILE" ]] || { echo "Map YAML does not exist: $MAP_FILE" >&2; exit 2; }; EXPLICIT_MAP=1 ;;
    --world) shift; [[ $# -gt 0 ]] || { echo '--world needs an SDF/world path' >&2; exit 2; }; WORLD_FILE="$(readlink -f -- "$1")"; [[ -f "$WORLD_FILE" ]] || { echo "Gazebo world does not exist: $WORLD_FILE" >&2; exit 2; }; EXPLICIT_WORLD=1 ;;
    --allow-dev-world) ALLOW_DEV_WORLD_SELECTED=true; ALLOW_DEV_WORLD_CLI=1 ;;
    --allow-dev-world=false) ALLOW_DEV_WORLD_SELECTED=false; ALLOW_DEV_WORLD_CLI=1 ;;
    --robot-id) shift; [[ $# -gt 0 ]] || { echo '--robot-id needs a value' >&2; exit 2; }; ROBOT_ID="$1"; [[ "$ROBOT_ID" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo 'robot id contains unsupported characters' >&2; exit 2; } ;;
    --namespace) shift; [[ $# -gt 0 ]] || { echo '--namespace needs a value' >&2; exit 2; }; NAMESPACE="${1#/}"; NAMESPACE="${NAMESPACE%/}"; [[ -z "$NAMESPACE" || "$NAMESPACE" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo 'namespace contains unsupported characters' >&2; exit 2; } ;;
    -h|--help) sed -n '1,100p' "$ROOT_DIR/scripts/start_stack.sh"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

# VMware-friendly defaults: keep the simulator server and the ROS stack while
# omitting both desktop rendering processes unless explicitly requested.

[[ -f "$ROOT_DIR/waretwin/backend/.env" ]] || { echo 'Missing backend/.env; run setup_full_stack.sh first' >&2; exit 1; }
[[ -f "$ROOT_DIR/waretwin/frontend/.env" ]] || { echo 'Missing frontend/.env; run setup_full_stack.sh first' >&2; exit 1; }
# The backend dotenv is the launch source of truth.  In particular, do not let
# a stale ROS_DOMAIN_ID exported by an IDE terminal silently select a different
# DDS domain than the backend and the runtime status file.
set -a
# shellcheck disable=SC1091
source "$ROOT_DIR/waretwin/backend/.env"
set +a
# This entrypoint starts the real Gazebo/ROS stack, so neither a stale shell
# variable nor backend/.env may silently select LOCAL_SIM for Django.
WARETWIN_RUNTIME_MODE=GAZEBO_ROS
export WARETWIN_RUNTIME_MODE
# Load the canonical ROS underlay, workspace overlay, and DDS settings before
# snapshotting the environment that every stack process and CLI probe will use.
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/ros_env.sh"
set -u
if ((ALLOW_DEV_WORLD_CLI == 0)); then
  ALLOW_DEV_WORLD_SELECTED="${WARETWIN_ALLOW_DEV_WORLD:-${ALLOW_DEV_WORLD:-$ALLOW_DEV_WORLD_SELECTED}}"
fi
case "${ALLOW_DEV_WORLD_SELECTED,,}" in
  true|1|yes) ALLOW_DEV_WORLD_SELECTED=true ;;
  false|0|no) ALLOW_DEV_WORLD_SELECTED=false ;;
  *) echo "Invalid ALLOW_DEV_WORLD=$ALLOW_DEV_WORLD_SELECTED; use true or false" >&2; exit 2 ;;
esac
ROS_DOMAIN_ID_SELECTED="${ROS_DOMAIN_ID:-0}"
if ! stack_valid_ros_domain "$ROS_DOMAIN_ID_SELECTED"; then
  echo "Invalid ROS_DOMAIN_ID=$ROS_DOMAIN_ID_SELECTED; use an integer from 0 to 232" >&2
  exit 2
fi

if stack_owned_pid backend || stack_owned_pid frontend || stack_owned_pid ros || stack_owned_pid ros_bridge; then
  echo 'A WareTwin stack process is already running. Use status_stack.sh or stop_stack.sh first.' >&2
  exit 1
fi
if stack_owned_group backend || stack_owned_group frontend || stack_owned_group ros || stack_owned_group ros_bridge; then
  echo 'A WareTwin stack process group is already running. Use stop_stack.sh first.' >&2
  exit 1
fi

requested_backend="${BACKEND_PORT_ARG:-${BACKEND_PORT:-8000}}"
requested_frontend="${FRONTEND_PORT_ARG:-${FRONTEND_PORT:-5173}}"
if [[ -n "$BACKEND_PORT_ARG" ]] && stack_port_busy "$requested_backend"; then
  stack_port_owner_message "$requested_backend" >&2
  exit 1
fi
if [[ -n "$FRONTEND_PORT_ARG" ]] && stack_port_busy "$requested_frontend"; then
  stack_port_owner_message "$requested_frontend" >&2
  exit 1
fi
BACKEND_PORT_SELECTED="$(stack_choose_port "$requested_backend" 8001)" || { echo 'No free backend port found' >&2; exit 1; }
FRONTEND_PORT_SELECTED="$(stack_choose_port "$requested_frontend" 5174)" || { echo 'No free frontend port found' >&2; exit 1; }

if [[ "$BACKEND_PORT_SELECTED" != "$requested_backend" ]]; then
  echo "[WARN] backend port $requested_backend is occupied; using $BACKEND_PORT_SELECTED"
  stack_port_owner_message "$requested_backend"
fi
if [[ "$FRONTEND_PORT_SELECTED" != "$requested_frontend" ]]; then
  echo "[WARN] frontend port $requested_frontend is occupied; using $FRONTEND_PORT_SELECTED"
  stack_port_owner_message "$requested_frontend"
fi

FRONTEND_HOST_SELECTED="${FRONTEND_HOST:-127.0.0.1}"
BACKEND_HOST_SELECTED="${BACKEND_HOST:-127.0.0.1}"
BACKEND_READY_TIMEOUT_S="${WARETWIN_BACKEND_READY_TIMEOUT_S:-120}"
if ! [[ "$BACKEND_READY_TIMEOUT_S" =~ ^[0-9]+$ ]] || (( BACKEND_READY_TIMEOUT_S < 1 )); then
  echo "Invalid WARETWIN_BACKEND_READY_TIMEOUT_S=$BACKEND_READY_TIMEOUT_S; use a positive integer" >&2
  exit 2
fi
BACKEND_URL="http://127.0.0.1:$BACKEND_PORT_SELECTED"
ROS_WS_URL_SELECTED="ws://127.0.0.1:$BACKEND_PORT_SELECTED/ws/ros"
FRONTEND_URL="http://127.0.0.1:$FRONTEND_PORT_SELECTED"
ALLOWED_HOSTS_SELECTED="$(stack_allowed_hosts "${DJANGO_ALLOWED_HOSTS:-}")"
CORS_SELECTED="$(stack_cors_origins "$FRONTEND_PORT_SELECTED" "${CORS_ALLOWED_ORIGINS:-}")"

umask 077
cat > "$STACK_RUNTIME_DIR/stack.env" <<EOF
MODE=$MODE
BACKEND_PORT=$BACKEND_PORT_SELECTED
FRONTEND_PORT=$FRONTEND_PORT_SELECTED
BACKEND_URL=$BACKEND_URL
FRONTEND_URL=$FRONTEND_URL
ROS_WS_URL=$ROS_WS_URL_SELECTED
MAP_FILE=$MAP_FILE
ROBOT_ID=$ROBOT_ID
NAMESPACE=$NAMESPACE
ROS_DOMAIN_ID=$ROS_DOMAIN_ID_SELECTED
ROS_DOMAIN_ID_SOURCE=backend/.env
RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION
ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY
FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-}
WARETWIN_RUNTIME_MODE=$WARETWIN_RUNTIME_MODE
GAZEBO_GUI=$GUI_ARG
RVIZ=$RVIZ_ARG
EOF

FRONTEND_MODE_SELECTED="${WARETWIN_FRONTEND_MODE:-production}"
case "$FRONTEND_MODE_SELECTED" in
  production)
    echo 'Building production frontend (React development validation is expensive on the VM)'
    (cd "$ROOT_DIR/waretwin/frontend" && env VITE_RUNTIME_MODE=GAZEBO_ROS VITE_DEMO_MODE=false VITE_BACKEND_MODE=true VITE_BACKEND_PORT="$BACKEND_PORT_SELECTED" VITE_API_BASE_URL= VITE_WS_BASE_URL= npm run build)
    FRONTEND_RUN_SCRIPT=preview
    ;;
  development) FRONTEND_RUN_SCRIPT=dev ;;
  *) echo 'WARETWIN_FRONTEND_MODE must be production or development' >&2; exit 2 ;;
esac

echo "Starting backend on $BACKEND_URL"
setsid env BACKEND_HOST="$BACKEND_HOST_SELECTED" BACKEND_PORT="$BACKEND_PORT_SELECTED" \
  WARETWIN_RUNTIME_MODE=GAZEBO_ROS \
  WARETWIN_STACK_RUNTIME_DIR="$STACK_RUNTIME_DIR" \
  DJANGO_ALLOWED_HOSTS="$ALLOWED_HOSTS_SELECTED" CORS_ALLOWED_ORIGINS="$CORS_SELECTED" \
  bash -c "cd '$ROOT_DIR/waretwin/backend' && exec ./run.sh" \
  > "$(stack_log_file backend)" 2>&1 < /dev/null &
stack_write_pid backend "$!"

if ! stack_wait_http "$BACKEND_URL/api/health/" "$BACKEND_READY_TIMEOUT_S"; then
  echo "Backend did not become healthy; see $(stack_log_file backend)" >&2
  stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 1
fi

# A published map bundle is mandatory unless development fallback was explicitly
# enabled. Never combine an explicit world/map with a different published map.
PUBLISHED_ARTIFACT_DIR=""
PUBLISHED_TAG_FILE=""
PUBLISHED_GRAPH_FILE=""
PUBLISHED_REVISION=0
SPAWN_TEXT="N/A (development world)"
ALLOW_DEV_WORLD_ARG=false
[[ "$ALLOW_DEV_WORLD_SELECTED" == true ]] && ALLOW_DEV_WORLD_ARG=true
MAP_SYNC_REQUEST_FILE="$STACK_RUNTIME_DIR/map-sync-request.json"
MODE_SWITCH_REQUEST_FILE="$STACK_RUNTIME_DIR/mode-switch-request.json"
MODE_SWITCH_STATUS_FILE="$STACK_RUNTIME_DIR/mode-switch-status.json"
DEVELOPMENT_WORLD=0

if ((EXPLICIT_MAP == 1 && EXPLICIT_WORLD == 0)) && [[ "$ALLOW_DEV_WORLD_SELECTED" != true ]]; then
  # A CLI map that is the currently published canonical map remains part of
  # its verified bundle and can use the paired world without dev-world opt-in.
  BACKEND_PYTHON="$ROOT_DIR/waretwin/backend/.venv/bin/python"
  CANONICAL_LINE=""
  if [[ -x "$BACKEND_PYTHON" ]]; then
    CANONICAL_LINE="$({
      cd "$ROOT_DIR/waretwin/backend"
      "$BACKEND_PYTHON" manage.py shell --verbosity 0 -c \
        'from twin.map_sync import published_map_payload; p=published_map_payload(); print("WARETWIN_ARTIFACTS|" + str(p.get("artifact_dir") or "") + "|" + str(p.get("map_revision") or ""))'
    } 2>/dev/null | rg '^WARETWIN_ARTIFACTS[|]' | tail -n1 || true)"
  fi
  if [[ "$CANONICAL_LINE" == *'|'* ]]; then
    CANONICAL_ARTIFACT_DIR="$(cut -d'|' -f2 <<<"$CANONICAL_LINE")"
    CANONICAL_REVISION="$(cut -d'|' -f3 <<<"$CANONICAL_LINE")"
    if [[ -n "$CANONICAL_ARTIFACT_DIR" && "$CANONICAL_REVISION" =~ ^[0-9]+$ ]]; then
      if CANONICAL_MAP="$(python3 - "$ROOT_DIR" "$CANONICAL_ARTIFACT_DIR" "$CANONICAL_REVISION" "$ROBOT_ID" 2>/dev/null <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]) / 'scripts'))
from ros_stack_supervisor import verify_bundle
_, selected = verify_bundle(Path(sys.argv[2]), int(sys.argv[3]), sys.argv[4])
print(selected['map'])
PY
      )"; then
        if [[ "$(readlink -f -- "$MAP_FILE")" == "$(readlink -f -- "$CANONICAL_MAP")" ]]; then
          EXPLICIT_MAP=0
          echo "[MAP] --map matches the verified published bundle: revision=$CANONICAL_REVISION"
        fi
      fi
    fi
  fi
fi

if ((EXPLICIT_MAP == 1 || EXPLICIT_WORLD == 1)); then
  if [[ "$ALLOW_DEV_WORLD_SELECTED" != true ]]; then
    echo 'Explicit --world/--map selects development assets; pass --allow-dev-world to permit them.' >&2
    stack_kill_owned backend
    rm -f "$STACK_RUNTIME_DIR/stack.env"
    exit 2
  fi
  DEVELOPMENT_WORLD=1
  [[ -n "$MAP_FILE" ]] || MAP_FILE="$ROOT_DIR/swerve_navigation/maps/warehouse.yaml"
elif [[ -n "$WORLD_FILE" ]]; then
  echo 'Internal error: world selection bypassed the canonical map selector.' >&2
  stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 2
else
  BACKEND_PYTHON="$ROOT_DIR/waretwin/backend/.venv/bin/python"
  PUBLISHED_LINE=""
  if [[ -x "$BACKEND_PYTHON" ]]; then
    PUBLISHED_LINE="$({
      cd "$ROOT_DIR/waretwin/backend"
      "$BACKEND_PYTHON" manage.py shell --verbosity 0 -c \
        'from twin.map_sync import published_map_payload; p=published_map_payload(); print("WARETWIN_ARTIFACTS|" + str(p.get("artifact_dir") or "") + "|" + str(p.get("map_revision") or ""))'
    } 2>/dev/null | rg '^WARETWIN_ARTIFACTS\|' | tail -n1 || true)"
  fi
  if [[ "$PUBLISHED_LINE" == *'|'* ]]; then
    PUBLISHED_ARTIFACT_DIR="$(cut -d'|' -f2 <<<"$PUBLISHED_LINE")"
    PUBLISHED_REVISION="$(cut -d'|' -f3 <<<"$PUBLISHED_LINE")"
  fi
  BUNDLE_OUTPUT=""
  BUNDLE_VALIDATION_ERROR=""
  if [[ -n "$PUBLISHED_ARTIFACT_DIR" && "$PUBLISHED_REVISION" =~ ^[0-9]+$ ]]; then
    if BUNDLE_OUTPUT="$(python3 - "$ROOT_DIR" "$PUBLISHED_ARTIFACT_DIR" "$PUBLISHED_REVISION" "$ROBOT_ID" 2>&1 <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]) / 'scripts'))
from ros_stack_supervisor import verify_bundle
root = Path(sys.argv[2])
revision = int(sys.argv[3])
manifest, selected = verify_bundle(root, revision, sys.argv[4])
print(selected['world'])
print(selected['map'])
print(selected['datamatrix'])
print(selected['graph'])
print(','.join(str(value) for value in selected['spawn']))
PY
    )"; then
      :
    else
      BUNDLE_VALIDATION_ERROR="$BUNDLE_OUTPUT"
      BUNDLE_OUTPUT=""
    fi
  fi
  if [[ -n "$BUNDLE_OUTPUT" ]]; then
    mapfile -t BUNDLE_FIELDS <<<"$BUNDLE_OUTPUT"
    WORLD_FILE="${BUNDLE_FIELDS[0]}"
    MAP_FILE="${BUNDLE_FIELDS[1]}"
    PUBLISHED_TAG_FILE="${BUNDLE_FIELDS[2]}"
    PUBLISHED_GRAPH_FILE="${BUNDLE_FIELDS[3]}"
    SPAWN_TEXT="${BUNDLE_FIELDS[4]}"
    ALLOW_DEV_WORLD_ARG=false
    echo "[MAP] Published canonical bundle verified: revision=$PUBLISHED_REVISION"
  elif [[ "$ALLOW_DEV_WORLD_SELECTED" == true ]]; then
    DEVELOPMENT_WORLD=1
    PUBLISHED_REVISION=0
    WORLD_FILE=""
    MAP_FILE="$ROOT_DIR/swerve_navigation/maps/warehouse.yaml"
    echo "[WARN] Published map bundle is unavailable/invalid; explicit development fallback enabled."
  else
    echo "[FAIL] No valid published map bundle for robot $ROBOT_ID. Publish a valid map or explicitly pass --allow-dev-world." >&2
    [[ -n "$BUNDLE_VALIDATION_ERROR" ]] && printf '%s\n' "$BUNDLE_VALIDATION_ERROR" >&2
    stack_kill_owned backend
    rm -f "$STACK_RUNTIME_DIR/stack.env"
    exit 1
  fi
fi

# Runtime requests are launch-owned state. A fresh start already resolves the
# currently published revision, so any prior request is obsolete.
rm -f "$MAP_SYNC_REQUEST_FILE" "$MAP_SYNC_REQUEST_FILE.tmp" \
  "$MODE_SWITCH_REQUEST_FILE" "$MODE_SWITCH_REQUEST_FILE.tmp"
cat > "$MODE_SWITCH_STATUS_FILE" <<EOF
{"robot_id":"$ROBOT_ID","mode":"$MODE","status":"STARTING","message":"initial runtime readiness is in progress"}
EOF
cat > "$STACK_RUNTIME_DIR/stack.env" <<EOF
MODE=$MODE
BACKEND_PORT=$BACKEND_PORT_SELECTED
FRONTEND_PORT=$FRONTEND_PORT_SELECTED
BACKEND_URL=$BACKEND_URL
FRONTEND_URL=$FRONTEND_URL
ROS_WS_URL=$ROS_WS_URL_SELECTED
MAP_FILE=$MAP_FILE
WORLD_FILE=$WORLD_FILE
MAP_REVISION=$PUBLISHED_REVISION
ROBOT_ID=$ROBOT_ID
NAMESPACE=$NAMESPACE
ROS_DOMAIN_ID=$ROS_DOMAIN_ID_SELECTED
ROS_DOMAIN_ID_SOURCE=backend/.env
RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION
ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY
FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-}
WARETWIN_RUNTIME_MODE=$WARETWIN_RUNTIME_MODE
ALLOW_DEV_WORLD=$ALLOW_DEV_WORLD_ARG
GAZEBO_GUI=$GUI_ARG
RVIZ=$RVIZ_ARG
EOF
echo "[MAP] revision=$PUBLISHED_REVISION frame=map canonical=${PUBLISHED_ARTIFACT_DIR:-N/A}"
echo "[GAZEBO] world=${WORLD_FILE:-$ROOT_DIR/worlds/warehouse.world}"
echo "[NAV2] map=${MAP_FILE:-package default (development only)}"
echo "[ROBOT] id=$ROBOT_ID spawn=($SPAWN_TEXT)"
echo "[ROS_DOMAIN_ID] $ROS_DOMAIN_ID_SELECTED"

echo "Starting frontend on $FRONTEND_URL"
setsid bash -c "cd '$ROOT_DIR/waretwin/frontend' && exec env VITE_RUNTIME_MODE=GAZEBO_ROS VITE_DEMO_MODE=false VITE_BACKEND_MODE=true VITE_BACKEND_PORT='$BACKEND_PORT_SELECTED' VITE_API_BASE_URL= VITE_WS_BASE_URL= npm run '$FRONTEND_RUN_SCRIPT' -- --host '$FRONTEND_HOST_SELECTED' --port '$FRONTEND_PORT_SELECTED' --strictPort" \
  > "$(stack_log_file frontend)" 2>&1 < /dev/null &
stack_write_pid frontend "$!"
if ! stack_wait_http "$FRONTEND_URL" 30; then
  echo "Frontend did not become ready; see $(stack_log_file frontend)" >&2
  stack_kill_owned frontend
  stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 1
fi

ROS_ARGS=(use_sim:=true use_sim_time:=true mode:="$MODE" gui:="$GUI_ARG" start_rviz:="$RVIZ_ARG" robot_id:="$ROBOT_ID" bridge_ws_url:="$ROS_WS_URL_SELECTED" allow_dev_world:="$ALLOW_DEV_WORLD_ARG")
if [[ "$MODE" == navigation || "$MODE" == unified ]]; then
  # Let the readiness probe start Nav2 only after Gazebo, ros2_control, SLAM,
  # TF, and sensor data are live; autostart during cold world load can strand
  # controller_server's costmap activation before its live map/TF exists.
  ROS_ARGS+=(defer_nav2_start:=true)
fi
[[ -n "$NAMESPACE" ]] && ROS_ARGS+=(namespace:="$NAMESPACE")
if [[ -n "$WORLD_FILE" ]]; then ROS_ARGS+=(world:="$WORLD_FILE"); fi
if [[ -n "$MAP_FILE" ]]; then ROS_ARGS+=(map_file:="$MAP_FILE"); fi
if [[ -n "$PUBLISHED_TAG_FILE" ]]; then ROS_ARGS+=(datamatrix_map_file:="$PUBLISHED_TAG_FILE"); fi
if [[ -n "$PUBLISHED_GRAPH_FILE" ]]; then ROS_ARGS+=(tag_graph_file:="$PUBLISHED_GRAPH_FILE"); fi
echo "Starting ROS/Gazebo in $MODE mode"
echo "Gazebo GUI=$GUI_ARG RViz=$RVIZ_ARG"
stack_prepare_log ros
stack_prepare_log ros_bridge
setsid bash -c '
  set -euo pipefail
  root="$1"; domain="$2"; ws_url="$3"; request_file="$4"; revision="$5"; robot_id="$6"
  ros_log="$7"; ros_bridge_log="$8"; mode_request="$9"; mode_status="${10}"
  initial_mode="${11}"; stack_env="${12}"; backend_url="${13}"; map_file="${14}"
  lifecycle_state_file="${15}"
  shift 15
  cd "$root"
  source scripts/ros_env.sh
  export ROS_DOMAIN_ID="$domain" ROS_WS_URL="$ws_url"
  export WARETWIN_NAV2_LIFECYCLE_STATE_FILE="$lifecycle_state_file"
  python3 scripts/ros_stack_supervisor.py --request-file "$request_file" \
    --initial-revision "$revision" --robot-id "$robot_id" \
    --mode-request-file "$mode_request" --mode-status-file "$mode_status" \
    --initial-mode "$initial_mode" --stack-env-file "$stack_env" \
    --readiness-root "$root" --backend-url "$backend_url" --map-file "$map_file" \
    -- "$@" 2>&1 \
    | tee "$ros_log" "$ros_bridge_log" >/dev/null
' _ "$ROOT_DIR" "$ROS_DOMAIN_ID_SELECTED" "$ROS_WS_URL_SELECTED" "$MAP_SYNC_REQUEST_FILE" \
  "$PUBLISHED_REVISION" "$ROBOT_ID" "$(stack_log_file ros)" "$(stack_log_file ros_bridge)" \
  "$MODE_SWITCH_REQUEST_FILE" "$MODE_SWITCH_STATUS_FILE" "$MODE" "$STACK_RUNTIME_DIR/stack.env" \
  "$BACKEND_URL" "${MAP_FILE:-}" "$NAV2_LIFECYCLE_STATE_FILE" \
  ros2 launch swerve_bringup system.launch.py "${ROS_ARGS[@]}" \
  > /dev/null 2>&1 < /dev/null &
stack_write_pid ros "$!"

# ros2 launch can fail while child processes such as Gazebo briefly remain
# alive. Wait for a launch process-start marker or a concrete launch error;
# then clean only this stack's process groups instead of reporting a false
# success.
ros_ready=0
for _ in $(seq 1 45); do
  if rg -q '\[ERROR\].*(Caught exception|package .+ not found|Unable to find package|failed)' "$(stack_log_file ros)"; then
    break
  fi
  gazebo_marker="$(rg -o '\[gzserver-[^]]+\]: process started with pid \[[0-9]+\]' \
    "$(stack_log_file ros)" | head -n1 || true)"
  if [[ -n "$gazebo_marker" ]]; then
    gazebo_pid="$(sed -nE 's/.*pid \[([0-9]+)\].*/\1/p' <<<"$gazebo_marker")"
    gazebo_comm=""
    if [[ -r "/proc/$gazebo_pid/comm" ]]; then
      IFS= read -r gazebo_comm < "/proc/$gazebo_pid/comm" || true
    fi
    if [[ "$gazebo_comm" == gzserver ]]; then
      read -r GAZEBO_STARTED_MONOTONIC_S _ < /proc/uptime
      export WARETWIN_GAZEBO_STARTED_MONOTONIC_S="$GAZEBO_STARTED_MONOTONIC_S"
      export WARETWIN_GAZEBO_PID="$gazebo_pid"
      echo "T1_GAZEBO_PROCESS_STARTED=PASS pid=$gazebo_pid monotonic_s=$GAZEBO_STARTED_MONOTONIC_S"
      ros_ready=1
      break
    fi
  fi
  if ! stack_owned_pid ros && ! stack_owned_group ros; then
    break
  fi
  sleep 1
done
if ((ros_ready == 0)); then
  echo "ROS/Gazebo launch failed; see $(stack_log_file ros)" >&2
  stack_kill_owned ros
  stack_kill_owned frontend
  stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 1
fi

echo '[OK] Stack processes started'
echo 'Waiting for ROS readiness...'

ros_readiness_report() {
  local probe_timeout="${1:-15}"
  (
    set +e
    export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_SELECTED"
    set +u
    source "$ROOT_DIR/scripts/ros_env.sh" || exit 1
    set -u
    readiness_args=(
      --mode "$MODE"
      --model swerve_base
      --robot-id "$ROBOT_ID"
      --timeout "$probe_timeout"
      --backend-url "$BACKEND_URL"
      --log-path "$(stack_log_file ros)"
      --lifecycle-state-file "$NAV2_LIFECYCLE_STATE_FILE"
    )
    if [[ ( "$MODE" == navigation || "$MODE" == unified ) && -n "$MAP_FILE" ]]; then
      readiness_args+=(--map-file "$MAP_FILE")
    fi
    # The Python probe owns its own deadline.  Wrapping it in GNU timeout sends
    # SIGTERM while rclpy is inside a wait set and turns a normal retry into a
    # misleading ExternalShutdownException traceback.
    python3 "$ROOT_DIR/scripts/navigation_readiness.py" "${readiness_args[@]}" 2>&1 \
      | tee -a "$(stack_log_file readiness)"
  )
}

# Nav2 and Gazebo initialize concurrently, but readiness still requires live
# samples at every critical stage. The previous 986 s SpawnEntity request
# coincided with kernel-reported VMware virtual-disk timeouts and ext4 journal
# I/O errors. A clean production-profile probe now inserts the robot in 28 s;
# cap retries at ten minutes so storage stalls fail safely instead of holding
# startup open for the old 30-minute allowance.
ROS_READY_TIMEOUT_S="${WARETWIN_ROS_READY_TIMEOUT_S:-600}"
if ! [[ "$ROS_READY_TIMEOUT_S" =~ ^[0-9]+$ ]] || (( ROS_READY_TIMEOUT_S < 1 )); then
  echo "Invalid WARETWIN_ROS_READY_TIMEOUT_S=$ROS_READY_TIMEOUT_S; use a positive integer" >&2
  stack_kill_owned ros; stack_kill_owned frontend; stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 2
fi
ros_ready=0
last_ros_report=''
: > "$(stack_log_file readiness)"
READINESS_PROBE_TIMEOUT_S="${WARETWIN_READINESS_PROBE_TIMEOUT_S:-120}"
if ! [[ "$READINESS_PROBE_TIMEOUT_S" =~ ^[0-9]+$ ]] || (( READINESS_PROBE_TIMEOUT_S < 2 )); then
  echo "Invalid WARETWIN_READINESS_PROBE_TIMEOUT_S=$READINESS_PROBE_TIMEOUT_S; use an integer >= 2" >&2
  stack_kill_owned ros || true
  stack_kill_owned frontend || true
  stack_kill_owned backend || true
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 2
fi
ready_deadline=$((SECONDS + ROS_READY_TIMEOUT_S))
while (( SECONDS < ready_deadline )); do
  remaining=$((ready_deadline - SECONDS))
  probe_timeout="$READINESS_PROBE_TIMEOUT_S"
  (( probe_timeout > remaining )) && probe_timeout="$remaining"
  if last_ros_report="$(ros_readiness_report "$probe_timeout")"; then
    ros_ready=1
    break
  fi
  if ! stack_owned_pid ros && ! stack_owned_group ros; then
    break
  fi
  if [[ ( "$MODE" == navigation || "$MODE" == unified ) && -f "$NAV2_LIFECYCLE_STATE_FILE" ]] \
    && rg -q '"state"[[:space:]]*:[[:space:]]*"FAILED"' "$NAV2_LIFECYCLE_STATE_FILE"; then
    echo '[FAIL] Nav2 lifecycle startup failed; refusing another STARTUP request for this launch.' >&2
    break
  fi
  sleep 2
done
if ((ros_ready == 0)); then
  echo "${last_ros_report:-[FAIL] readiness probe did not return a snapshot}"
  echo "[FAIL] ${MODE^} stack NOT READY; stopping only this stack's process groups and preserving logs." >&2
  cleanup_failed=0
  for component in ros frontend backend; do
    component_pgid="$(stack_pgid "$component" 2>/dev/null || true)"
    stack_kill_owned "$component" || cleanup_failed=1
    if stack_group_has_live_process "$component_pgid"; then
      echo "[FAIL] $component process group $component_pgid still has live processes" >&2
      cleanup_failed=1
    fi
  done
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  if ((cleanup_failed)); then
    echo "[FAIL] startup cleanup was incomplete; inspect $(stack_log_file ros) and $(stack_log_file ros_bridge)." >&2
  else
    echo "[OK] startup child processes stopped; logs preserved in $STACK_LOG_DIR." >&2
  fi
  exit 1
fi

cat > "$MODE_SWITCH_STATUS_FILE.tmp" <<EOF
{"robot_id":"$ROBOT_ID","mode":"$MODE","status":"READY","message":"initial runtime mode passed the existing readiness gate"}
EOF
mv -f "$MODE_SWITCH_STATUS_FILE.tmp" "$MODE_SWITCH_STATUS_FILE"
printf '%s\n' "$last_ros_report"
echo
if [[ "$MODE" == unified ]]; then
  echo '[OK] Unified SLAM + Nav2 stack READY'
elif [[ "$MODE" == mapping ]]; then
  echo '[OK] Legacy Mapping stack READY'
else
  echo '[OK] Navigation stack READY'
fi
echo "  frontend: $FRONTEND_URL"
echo "  backend:  $BACKEND_URL"
echo "  mode:     $MODE"
echo "  ros domain: $ROS_DOMAIN_ID_SELECTED"
echo "  logs:     $STACK_LOG_DIR/{backend,frontend,ros}.log"
echo "Use scripts/status_stack.sh for diagnostics and scripts/stop_stack.sh to stop only this stack."
