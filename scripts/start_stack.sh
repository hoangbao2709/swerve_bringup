#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"

MODE="${1:-mapping}"
if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  sed -n '1,125p' "$0"
  exit 0
fi
[[ "$MODE" == mapping || "$MODE" == navigation ]] || { echo "Usage: $0 {mapping|navigation} [options]" >&2; exit 2; }
shift || true

HEADLESS=0
NO_RVIZ=0
NO_GAZEBO_GUI=0
BACKEND_PORT_ARG=""
FRONTEND_PORT_ARG=""
MAP_FILE=""
WORLD_FILE=""
ROBOT_ID="${WARETWIN_ROBOT_ID:-R01}"
NAMESPACE="${WARETWIN_ROS_NAMESPACE:-}"

while (($#)); do
  case "$1" in
    --headless) HEADLESS=1; NO_RVIZ=1; NO_GAZEBO_GUI=1 ;;
    --no-rviz) NO_RVIZ=1 ;;
    --no-gazebo-gui) NO_GAZEBO_GUI=1 ;;
    --backend-port) shift; [[ $# -gt 0 ]] || { echo '--backend-port needs a value' >&2; exit 2; }; BACKEND_PORT_ARG="$1" ;;
    --frontend-port) shift; [[ $# -gt 0 ]] || { echo '--frontend-port needs a value' >&2; exit 2; }; FRONTEND_PORT_ARG="$1" ;;
    --map) shift; [[ $# -gt 0 ]] || { echo '--map needs a YAML path' >&2; exit 2; }; MAP_FILE="$(readlink -f -- "$1")"; [[ -f "$MAP_FILE" ]] || { echo "Map YAML does not exist: $MAP_FILE" >&2; exit 2; } ;;
    --world) shift; [[ $# -gt 0 ]] || { echo '--world needs an SDF/world path' >&2; exit 2; }; WORLD_FILE="$(readlink -f -- "$1")"; [[ -f "$WORLD_FILE" ]] || { echo "Gazebo world does not exist: $WORLD_FILE" >&2; exit 2; } ;;
    --robot-id) shift; [[ $# -gt 0 ]] || { echo '--robot-id needs a value' >&2; exit 2; }; ROBOT_ID="$1"; [[ "$ROBOT_ID" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo 'robot id contains unsupported characters' >&2; exit 2; } ;;
    --namespace) shift; [[ $# -gt 0 ]] || { echo '--namespace needs a value' >&2; exit 2; }; NAMESPACE="${1#/}"; NAMESPACE="${NAMESPACE%/}"; [[ -z "$NAMESPACE" || "$NAMESPACE" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo 'namespace contains unsupported characters' >&2; exit 2; } ;;
    -h|--help) sed -n '1,100p' "$ROOT_DIR/scripts/start_stack.sh"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

[[ -f "$ROOT_DIR/waretwin/backend/.env" ]] || { echo 'Missing backend/.env; run setup_full_stack.sh first' >&2; exit 1; }
[[ -f "$ROOT_DIR/waretwin/frontend/.env" ]] || { echo 'Missing frontend/.env; run setup_full_stack.sh first' >&2; exit 1; }
# The backend dotenv is the launch source of truth.  In particular, do not let
# a stale ROS_DOMAIN_ID exported by an IDE terminal silently select a different
# DDS domain than the backend and the runtime status file.
set -a
# shellcheck disable=SC1091
source "$ROOT_DIR/waretwin/backend/.env"
set +a
ROS_DOMAIN_ID_SELECTED="${ROS_DOMAIN_ID:-0}"
if ! stack_valid_ros_domain "$ROS_DOMAIN_ID_SELECTED"; then
  echo "Invalid ROS_DOMAIN_ID=$ROS_DOMAIN_ID_SELECTED; use an integer from 0 to 232" >&2
  exit 2
fi

if stack_owned_pid backend || stack_owned_pid frontend || stack_owned_pid ros; then
  echo 'A WareTwin stack process is already running. Use status_stack.sh or stop_stack.sh first.' >&2
  exit 1
fi
if stack_owned_group backend || stack_owned_group frontend || stack_owned_group ros; then
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

FRONTEND_HOST_SELECTED="${FRONTEND_HOST:-0.0.0.0}"
BACKEND_HOST_SELECTED="${BACKEND_HOST:-0.0.0.0}"
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
EOF

echo "Starting backend on $BACKEND_URL"
setsid env BACKEND_HOST="$BACKEND_HOST_SELECTED" BACKEND_PORT="$BACKEND_PORT_SELECTED" \
  DJANGO_ALLOWED_HOSTS="$ALLOWED_HOSTS_SELECTED" CORS_ALLOWED_ORIGINS="$CORS_SELECTED" \
  bash -c "cd '$ROOT_DIR/waretwin/backend' && exec ./run.sh" \
  > "$(stack_log_file backend)" 2>&1 < /dev/null &
stack_write_pid backend "$!"

if ! stack_wait_http "$BACKEND_URL/api/health/" 45; then
  echo "Backend did not become healthy; see $(stack_log_file backend)" >&2
  stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 1
fi

# A published canonical map is the only source for generated Gazebo geometry,
# tags and robot spawn poses. Resolve it after Django is ready so a clean start
# launches the same immutable revision that the editor published. An explicit
# --world always wins for development/test worlds.
PUBLISHED_ARTIFACT_DIR=""
PUBLISHED_TAG_FILE=""
PUBLISHED_GRAPH_FILE=""
if [[ -z "$WORLD_FILE" ]]; then
  BACKEND_PYTHON="$ROOT_DIR/waretwin/backend/.venv/bin/python"
  if [[ -x "$BACKEND_PYTHON" ]]; then
    PUBLISHED_LINE="$({
      cd "$ROOT_DIR/waretwin/backend"
      "$BACKEND_PYTHON" manage.py shell --verbosity 0 -c \
        'from twin.map_sync import published_map_payload; p=published_map_payload(); print("WARETWIN_ARTIFACTS|" + str(p.get("artifact_dir") or ""))'
    } 2>/dev/null | rg '^WARETWIN_ARTIFACTS\|' | tail -n1 || true)"
    PUBLISHED_ARTIFACT_DIR="${PUBLISHED_LINE#WARETWIN_ARTIFACTS|}"
    if [[ -n "$PUBLISHED_ARTIFACT_DIR" && -f "$PUBLISHED_ARTIFACT_DIR/gazebo/warehouse.world" ]]; then
      if python3 - "$PUBLISHED_ARTIFACT_DIR/manifest.json" "$ROBOT_ID" <<'PY'
import json
import sys

try:
    manifest = json.load(open(sys.argv[1], encoding='utf-8'))
    wanted = sys.argv[2]
    ok = any(str(robot.get('id')) == wanted for robot in manifest.get('robots', []))
except (OSError, ValueError, IndexError):
    ok = False
raise SystemExit(0 if ok else 1)
PY
      then
        WORLD_FILE="$PUBLISHED_ARTIFACT_DIR/gazebo/warehouse.world"
        PUBLISHED_TAG_FILE="$PUBLISHED_ARTIFACT_DIR/datamatrix_map.yaml"
        PUBLISHED_GRAPH_FILE="$PUBLISHED_ARTIFACT_DIR/tag_graph.yaml"
        echo "Using published canonical Gazebo revision: $WORLD_FILE"
      else
        echo "[WARN] Published artifact has no spawn record for robot $ROBOT_ID; using package development world. Publish a map with that robot or pass --world explicitly."
      fi
    else
      WORLD_FILE=""
      echo 'No published canonical Gazebo revision found; using package development world.'
    fi
  fi
fi

echo "Starting frontend on $FRONTEND_URL"
setsid bash -c "cd '$ROOT_DIR/waretwin/frontend' && exec env VITE_BACKEND_PORT='$BACKEND_PORT_SELECTED' VITE_API_BASE_URL= VITE_WS_BASE_URL= npm run dev -- --host '$FRONTEND_HOST_SELECTED' --port '$FRONTEND_PORT_SELECTED'" \
  > "$(stack_log_file frontend)" 2>&1 < /dev/null &
stack_write_pid frontend "$!"
if ! stack_wait_http "$FRONTEND_URL" 30; then
  echo "Frontend did not become ready; see $(stack_log_file frontend)" >&2
  stack_kill_owned frontend
  stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 1
fi

GUI_ARG=true
RVIZ_ARG=true
[[ "$NO_GAZEBO_GUI" -eq 1 ]] && GUI_ARG=false
[[ "$NO_RVIZ" -eq 1 ]] && RVIZ_ARG=false
ROS_ARGS=(use_sim:=true use_sim_time:=true mode:="$MODE" gui:="$GUI_ARG" start_rviz:="$RVIZ_ARG" robot_id:="$ROBOT_ID" bridge_ws_url:="$ROS_WS_URL_SELECTED")
[[ -n "$NAMESPACE" ]] && ROS_ARGS+=(namespace:="$NAMESPACE")
if [[ -n "$WORLD_FILE" ]]; then ROS_ARGS+=(world:="$WORLD_FILE"); fi
if [[ -n "$MAP_FILE" ]]; then ROS_ARGS+=(map_file:="$MAP_FILE"); fi
if [[ -n "$PUBLISHED_TAG_FILE" ]]; then ROS_ARGS+=(datamatrix_map_file:="$PUBLISHED_TAG_FILE"); fi
if [[ -n "$PUBLISHED_GRAPH_FILE" ]]; then ROS_ARGS+=(tag_graph_file:="$PUBLISHED_GRAPH_FILE"); fi
printf -v ROS_LAUNCH_ARGS '%q ' "${ROS_ARGS[@]}"

echo "Starting ROS/Gazebo in $MODE mode"
setsid bash -c "cd '$ROOT_DIR' && export ROS_DOMAIN_ID='$ROS_DOMAIN_ID_SELECTED' && source scripts/ros_env.sh && export ROS_DOMAIN_ID='$ROS_DOMAIN_ID_SELECTED' ROS_WS_URL='$ROS_WS_URL_SELECTED' && ros2 launch swerve_bringup system.launch.py ${ROS_LAUNCH_ARGS}" \
  > >(tee "$(stack_log_file ros)" "$(stack_log_file ros_bridge)" >/dev/null) 2>&1 < /dev/null &
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
  if rg -q 'process started with pid' "$(stack_log_file ros)"; then
    ros_ready=1
    break
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
    source "$ROOT_DIR/scripts/ros_env.sh" >/dev/null 2>&1 || exit 1
    readiness_args=(
      --mode "$MODE"
      --model swerve_base
      --timeout "$probe_timeout"
    )
    if [[ "$MODE" == navigation && -n "$MAP_FILE" ]]; then
      readiness_args+=(--map-file "$MAP_FILE")
    fi
    # The Python probe owns its own deadline.  Wrapping it in GNU timeout sends
    # SIGTERM while rclpy is inside a wait set and turns a normal retry into a
    # misleading ExternalShutdownException traceback.
    python3 "$ROOT_DIR/scripts/navigation_readiness.py" "${readiness_args[@]}"
  )
}

# Nav2 and Gazebo initialize concurrently. On a resource-constrained VM the
# robot can be spawned after lifecycle servers have already started, so keep a
# generous finite gate rather than reporting a false startup failure.
ROS_READY_TIMEOUT_S="${WARETWIN_ROS_READY_TIMEOUT_S:-300}"
if ! [[ "$ROS_READY_TIMEOUT_S" =~ ^[0-9]+$ ]] || (( ROS_READY_TIMEOUT_S < 1 )); then
  echo "Invalid WARETWIN_ROS_READY_TIMEOUT_S=$ROS_READY_TIMEOUT_S; use a positive integer" >&2
  stack_kill_owned ros; stack_kill_owned frontend; stack_kill_owned backend
  rm -f "$STACK_RUNTIME_DIR/stack.env"
  exit 2
fi
ros_ready=0
last_ros_report=''
READINESS_PROBE_TIMEOUT_S="${WARETWIN_READINESS_PROBE_TIMEOUT_S:-15}"
if ! [[ "$READINESS_PROBE_TIMEOUT_S" =~ ^[0-9]+$ ]] || (( READINESS_PROBE_TIMEOUT_S < 2 )); then
  echo "Invalid WARETWIN_READINESS_PROBE_TIMEOUT_S=$READINESS_PROBE_TIMEOUT_S; use an integer >= 2" >&2
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
  sleep 2
done
if ((ros_ready == 0)); then
  echo "${last_ros_report:-[FAIL] readiness probe did not return a snapshot}"
  if ! stack_owned_pid ros && ! stack_owned_group ros; then
    echo "ROS/Gazebo launch exited during readiness; cleaning its remaining web processes." >&2
    stack_kill_owned frontend
    stack_kill_owned backend
    rm -f "$STACK_RUNTIME_DIR/stack.env"
  else
    echo "[FAIL] ${MODE^} stack NOT READY"
    echo "Stack processes are left running for debugging. See $(stack_log_file ros) and $(stack_log_file ros_bridge)." >&2
  fi
  exit 1
fi

printf '%s\n' "$last_ros_report"
echo
if [[ "$MODE" == mapping ]]; then
  echo '[OK] Mapping stack READY'
else
  echo '[OK] Navigation stack READY'
fi
echo "  frontend: $FRONTEND_URL"
echo "  backend:  $BACKEND_URL"
echo "  mode:     $MODE"
echo "  ros domain: $ROS_DOMAIN_ID_SELECTED"
echo "  logs:     $STACK_LOG_DIR/{backend,frontend,ros}.log"
echo "Use scripts/status_stack.sh for diagnostics and scripts/stop_stack.sh to stop only this stack."
