#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAILURES=0
WARNINGS=0

ok() { printf '[OK] %s\n' "$1"; }
warn() { WARNINGS=$((WARNINGS + 1)); printf '[WARN] %s\n' "$1"; }
fail() { FAILURES=$((FAILURES + 1)); printf '[FAIL] %s\n' "$1"; }

version_at_least() {
  local actual="$1" required="$2"
  [[ "$(printf '%s\n%s\n' "$required" "$actual" | sort -V | head -n1)" == "$required" ]]
}

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]]; then
    ok "Ubuntu 22.04"
  else
    fail "Ubuntu 22.04 required (detected ${PRETTY_NAME:-unknown})"
  fi
else
  fail "cannot detect operating system (/etc/os-release missing)"
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
  export AMENT_TRACE_SETUP_FILES=""
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  if command -v ros2 >/dev/null 2>&1; then
    ok "ROS 2 Humble"
  else
    fail "ROS 2 Humble setup exists but ros2 is not executable"
  fi
else
  fail "ROS 2 Humble not found at /opt/ros/humble"
fi

if command -v python3 >/dev/null 2>&1; then
  PYTHON_VERSION="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
  if version_at_least "$PYTHON_VERSION" 3.10; then ok "Python $PYTHON_VERSION"; else fail "Python >=3.10 required (detected $PYTHON_VERSION)"; fi
else
  fail "python3 not found"
fi

if command -v node >/dev/null 2>&1; then
  NODE_VERSION="$(node -p 'process.versions.node')"
  NODE_MAJOR="${NODE_VERSION%%.*}"
  if [[ "$NODE_MAJOR" =~ ^[0-9]+$ ]] && (( NODE_MAJOR >= 20 )); then
    ok "Node $NODE_VERSION"
  else
    fail "Node.js >=20 required (Node 22 LTS recommended; detected $NODE_VERSION). Install with nvm: nvm install 22 && nvm use 22"
  fi
else
  fail "Node.js >=20 not found; install Node 22 LTS"
fi

if command -v npm >/dev/null 2>&1; then
  NPM_VERSION="$(npm --version 2>/dev/null || true)"
  if [[ "$NPM_VERSION" =~ ^[0-9]+([.][0-9]+)*$ ]] && version_at_least "$NPM_VERSION" 10; then ok "npm $NPM_VERSION"; else fail "npm >=10 required (detected ${NPM_VERSION:-unknown})"; fi
else
  fail "npm not found"
fi

if command -v gazebo >/dev/null 2>&1 && ros2 pkg prefix gazebo_ros >/dev/null 2>&1; then
  ok "Gazebo Classic + gazebo_ros"
else
  fail "Gazebo Classic/gazebo_ros is missing; install ros-humble-gazebo-ros-pkgs"
fi

ROS_REQUIRED_PACKAGES=(
  gazebo_ros2_control robot_localization pointcloud_to_laserscan slam_toolbox
  nav2_bringup nav2_controller nav2_planner nav2_map_server nav2_behaviors
  nav2_bt_navigator nav2_waypoint_follower nav2_lifecycle_manager nav2_msgs
  controller_manager controller_manager_msgs rviz2
)
ROS_MISSING_PACKAGES=()
if command -v ros2 >/dev/null 2>&1; then
  for package in "${ROS_REQUIRED_PACKAGES[@]}"; do
    if ros2 pkg prefix "$package" >/dev/null 2>&1; then
      continue
    fi
    ROS_MISSING_PACKAGES+=("$package")
  done
  if ((${#ROS_MISSING_PACKAGES[@]} == 0)); then
    ok "ROS simulation/navigation dependencies"
  else
    fail "Missing ROS packages: ${ROS_MISSING_PACKAGES[*]}. Run ./scripts/setup_full_stack.sh"
  fi
else
  fail "Cannot inspect ROS packages because ros2 is unavailable"
fi

if command -v rosdep >/dev/null 2>&1; then ok "rosdep"; else fail "rosdep not found; install python3-rosdep"; fi
if command -v colcon >/dev/null 2>&1; then ok "colcon"; else fail "colcon not found; install python3-colcon-common-extensions"; fi
if command -v rg >/dev/null 2>&1; then ok "ripgrep"; else fail "ripgrep not found; install ripgrep"; fi

BACKEND_PYTHON="$ROOT_DIR/waretwin/backend/.venv/bin/python"
if [[ -x "$BACKEND_PYTHON" ]] && "$BACKEND_PYTHON" -c 'import django, channels, daphne, pydantic, websocket' >/dev/null 2>&1; then
  ok "Backend dependencies"
else
  fail "Backend virtualenv/dependencies missing; run ./scripts/setup_full_stack.sh"
fi

if [[ -d "$ROOT_DIR/waretwin/frontend/node_modules" ]] && [[ -f "$ROOT_DIR/waretwin/frontend/node_modules/.package-lock.json" ]]; then
  ok "Frontend dependencies"
else
  fail "Frontend node_modules missing; run ./scripts/setup_full_stack.sh"
fi
if [[ -f "$ROOT_DIR/waretwin/frontend/.env" ]] && \
   grep -Eq '^VITE_DEMO_MODE=false([[:space:]]*#.*)?$' "$ROOT_DIR/waretwin/frontend/.env" && \
   grep -Eq '^VITE_RUNTIME_MODE=(GAZEBO_ROS|REAL_ROBOT)$' "$ROOT_DIR/waretwin/frontend/.env"; then
  ok "Frontend backend-connected environment"
else
  fail "Frontend .env must set VITE_DEMO_MODE=false and VITE_RUNTIME_MODE=GAZEBO_ROS (run setup_full_stack.sh)"
fi

if [[ -f "$ROOT_DIR/install/local_setup.bash" ]] && command -v ros2 >/dev/null 2>&1 \
   && { set +u; source "$ROOT_DIR/install/local_setup.bash"; set -u; } \
   && ros2 pkg prefix swerve_bringup >/dev/null 2>&1 \
   && ros2 pkg prefix swerve_bridge >/dev/null 2>&1; then
  ok "ROS workspace"
else
  fail "ROS workspace is not built; run ./scripts/build_ros.sh"
fi

if [[ -x "$BACKEND_PYTHON" ]] && (cd "$ROOT_DIR/waretwin/backend" && "$BACKEND_PYTHON" manage.py check --deploy >/dev/null 2>&1); then
  ok "Database/Django checks"
else
  warn "Django deploy checks are not clean; run manage.py check for details"
fi

if [[ -f "$ROOT_DIR/waretwin/backend/.env" ]]; then
  if grep -Eq '^WARETWIN_ROS_BRIDGE_TOKEN=[^[:space:]]+$' "$ROOT_DIR/waretwin/backend/.env" \
     && ! grep -Eq '^WARETWIN_ROS_BRIDGE_TOKEN=(change-me|)$' "$ROOT_DIR/waretwin/backend/.env"; then
    ok "Environment"
  else
    fail "backend/.env exists but WARETWIN_ROS_BRIDGE_TOKEN is empty/default"
  fi
else
  fail "waretwin/backend/.env is missing; copy .env.example and set a bridge token"
fi

port_state() {
  local port="$1"
  if command -v ss >/dev/null 2>&1 && ss -ltnH | awk -v p=":$port" '$4 ~ p"$" { found=1 } END { exit !found }'; then
    warn "Port $port is already in use; start_stack will choose a fallback or report the owner"
  else
    ok "Port $port is available"
  fi
}
port_state "${BACKEND_PORT:-8000}"
port_state "${FRONTEND_PORT:-5173}"

printf '\nPreflight summary: %d failure(s), %d warning(s)\n' "$FAILURES" "$WARNINGS"
if (( FAILURES > 0 )); then
  printf 'Fix the [FAIL] items, then run this check again.\n'
  exit 1
fi
exit 0
