#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/setup.log"
exec > >(tee -a "$LOG_FILE") 2>&1

fail() {
  echo "[ERROR] $*" >&2
  echo "Setup failed. See $LOG_FILE" >&2
  exit 1
}

if [[ ! -r /etc/os-release ]]; then
  fail 'cannot detect Ubuntu version'
fi
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  fail "Ubuntu 22.04 is required (detected ${PRETTY_NAME:-unknown})"

# Node is intentionally validated before any privileged package operation.  It
# is not installed by the ROS apt bundle, so a Node mismatch should produce the
# actionable version error immediately even on a machine where sudo is not
# available in the current terminal.
command -v node >/dev/null 2>&1 || fail 'Node.js >=20 is required; install Node 22 LTS before rerunning setup'
NODE_VERSION="$(node -p 'process.versions.node')"
NODE_MAJOR="${NODE_VERSION%%.*}"
if ! [[ "$NODE_MAJOR" =~ ^[0-9]+$ ]] || ((NODE_MAJOR < 20)); then
  fail "Node.js >=20 is required (Node 22 LTS recommended; detected $NODE_VERSION). Install with nvm: nvm install 22 && nvm use 22"
fi
command -v npm >/dev/null 2>&1 || fail 'npm is missing; install Node 22 LTS before rerunning setup'
NPM_VERSION="$(npm --version)"
NPM_MAJOR="${NPM_VERSION%%.*}"
if ! [[ "$NPM_MAJOR" =~ ^[0-9]+$ ]] || ((NPM_MAJOR < 10)); then
  fail "npm >=10 is required (detected $NPM_VERSION). Upgrade Node.js/npm and rerun setup"
fi

SUDO=()
SUDO_READY=0
ensure_sudo() {
  ((SUDO_READY)) && return 0
  if [[ "${EUID}" -eq 0 ]]; then
    SUDO=()
    SUDO_READY=1
    return 0
  fi
  command -v sudo >/dev/null 2>&1 || fail 'sudo is required for the privileged operation that is about to run'
  sudo -v || fail 'sudo authentication failed; re-run with an account allowed to install packages'
  SUDO=(sudo)
  SUDO_READY=1
}

ROS_APT_PACKAGES=(
  ros-humble-gazebo-ros-pkgs
  ros-humble-gazebo-ros2-control
  ros-humble-gazebo-plugins
  ros-humble-ros2-controllers
  ros-humble-joint-state-broadcaster
  ros-humble-position-controllers
  ros-humble-velocity-controllers
  ros-humble-controller-manager
  ros-humble-robot-localization
  ros-humble-pointcloud-to-laserscan
  ros-humble-slam-toolbox
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-nav2-controller
  ros-humble-nav2-planner
  ros-humble-nav2-map-server
  ros-humble-nav2-behaviors
  ros-humble-nav2-bt-navigator
  ros-humble-nav2-waypoint-follower
  ros-humble-nav2-lifecycle-manager
  ros-humble-nav2-msgs
  ros-humble-ros2-control
  ros-humble-rviz2
  ros-humble-xacro
  ros-humble-joint-state-publisher
  ros-humble-joint-state-publisher-gui
  ros-humble-tf2-tools
  ros-humble-diagnostic-updater
  python3-colcon-common-extensions
  python3-rosdep
  python3-venv
  python3-yaml
  python3-websocket
  python3-psutil
  curl
  lsof
  ripgrep
)

missing=()
for package in "${ROS_APT_PACKAGES[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then
    missing+=("$package")
  fi
done

# A package can be marked installed while an older ROS archive is ABI
# incompatible with a newer dependent package.  In particular, newer
# robot_localization binaries require the C++ diagnostic_updater shared
# library, whereas older Humble archives only shipped the headers/Python
# module.  Treat that missing runtime artifact as a real repair operation.
runtime_repairs=()
if [[ ! -f /opt/ros/humble/lib/libdiagnostic_updater.so ]] \
   && dpkg-query -W -f='${Status}' ros-humble-diagnostic-updater 2>/dev/null | grep -q 'install ok installed'; then
  runtime_repairs+=(ros-humble-diagnostic-updater)
fi

if ((${#missing[@]} || ${#runtime_repairs[@]})); then
  echo "Installing ${#missing[@]} missing system/ROS package(s) and repairing ${#runtime_repairs[@]} runtime package(s)..."
  ensure_sudo
  "${SUDO[@]}" apt-get update
  if ((${#missing[@]})); then
    "${SUDO[@]}" apt-get install -y "${missing[@]}"
  fi
  if ((${#runtime_repairs[@]})); then
    "${SUDO[@]}" apt-get install -y --reinstall "${runtime_repairs[@]}"
  fi
else
  echo 'All declared Ubuntu/ROS packages are already installed.'
fi

[[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble not found at /opt/ros/humble'
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH AMENT_TRACE_SETUP_FILES COLCON_TRACE
export AMENT_TRACE_SETUP_FILES=""
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u

command -v python3 >/dev/null 2>&1 || fail 'python3 is missing'
command -v gazebo >/dev/null 2>&1 || fail 'Gazebo Classic executable is missing'
command -v colcon >/dev/null 2>&1 || fail 'colcon is missing'
command -v rosdep >/dev/null 2>&1 || fail 'rosdep is missing'

if ! rosdep db >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    ensure_sudo
    "${SUDO[@]}" rosdep init
  fi
  rosdep update
fi

cd "$ROOT_DIR"
echo 'Resolving ROS package dependencies with rosdep...'
rosdep install --from-paths "$ROOT_DIR" "$ROOT_DIR/swerve_bridge" \
  --ignore-src --rosdistro humble -r -y

BACKEND_DIR="$ROOT_DIR/waretwin/backend"
cd "$BACKEND_DIR"
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
fi
backend_python() {
  # A parent ROS overlay may export PYTHONPATH entries with unrelated package
  # metadata. Keep backend dependency checks/install isolated and repeatable.
  env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH .venv/bin/python "$@"
}
if ! backend_python -c 'import django, channels, daphne, dotenv, pydantic, websocket, wsaccel, openpyxl, pytest, yaml' >/dev/null 2>&1 \
   || ! backend_python -m pip check >/dev/null 2>&1; then
  backend_python -m pip install --disable-pip-version-check -r requirements-dev.txt
else
  echo 'Backend Python dependencies are already installed and consistent.'
fi
if [[ ! -f .env ]]; then
  cp .env.example .env
fi

set_env_default() {
  local key="$1" value="$2"
  if grep -qE "^${key}=" .env; then
    return 0
  fi
  printf '%s=%s\n' "$key" "$value" >> .env
}

set_env_default BACKEND_HOST 0.0.0.0
set_env_default BACKEND_PORT 8000
set_env_default FRONTEND_HOST 0.0.0.0
set_env_default FRONTEND_PORT 5173
set_env_default ROS_DOMAIN_ID 0
set_env_default ROS_WS_URL ws://127.0.0.1:8000/ws/ros
set_env_default WARETWIN_ARTIFACT_ROOT "$ROOT_DIR/generated/maps"
set_env_default TWIN_ADMIN_PASSWORD ""
if grep -qE '^TWIN_ADMIN_PASSWORD=(|change-me-before-first-run)$' .env; then
  # Store a one-time local bootstrap credential in the ignored .env file. It is
  # deliberately not printed to setup.log; operators can rotate it afterwards.
  _admin_password="$(python3 -c 'import secrets; print(secrets.token_urlsafe(24))')"
  sed -i "s|^TWIN_ADMIN_PASSWORD=.*|TWIN_ADMIN_PASSWORD=$_admin_password|" .env
  unset _admin_password
  echo 'Generated a local TWIN_ADMIN_PASSWORD in waretwin/backend/.env (not printed).'
fi
if grep -qE '^WARETWIN_ROS_BRIDGE_TOKEN=(change-me|)$' .env; then
  TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
  sed -i "s|^WARETWIN_ROS_BRIDGE_TOKEN=.*|WARETWIN_ROS_BRIDGE_TOKEN=$TOKEN|" .env
fi
if grep -qE '^DJANGO_SECRET_KEY=change-me-in-production$' .env; then
  SECRET_KEY="$(python3 -c 'import secrets; print(secrets.token_urlsafe(48))')"
  sed -i "s|^DJANGO_SECRET_KEY=.*|DJANGO_SECRET_KEY=$SECRET_KEY|" .env
fi

set -a
# shellcheck disable=SC1091
source .env
set +a

backend_python manage.py migrate --noinput
backend_python manage.py seed_demo
backend_python manage.py sync_master_data
backend_python manage.py check
backend_python manage.py makemigrations --check --dry-run

FRONTEND_DIR="$ROOT_DIR/waretwin/frontend"
cd "$FRONTEND_DIR"
if [[ ! -f .env ]]; then
  cp .env.example .env
fi
if [[ ! -d node_modules || ! -f node_modules/.package-lock.json || package-lock.json -nt node_modules/.package-lock.json ]]; then
  npm ci --no-audit --no-fund
else
  npm ls --depth=0 >/dev/null 2>&1 || npm ci --no-audit --no-fund
fi
npm run build
npm test

cd "$ROOT_DIR"
scripts/build_ros.sh
source scripts/ros_env.sh
scripts/preflight_check.sh

cat <<EOF

[OK] Full stack setup complete.
Backend:  http://127.0.0.1:${BACKEND_PORT:-8000}
Frontend: http://127.0.0.1:${FRONTEND_PORT:-5173}
ROS:      source $ROOT_DIR/scripts/ros_env.sh
Logs:     $LOG_FILE

Start manually:
  cd $ROOT_DIR/waretwin/backend && ./run.sh
  cd $ROOT_DIR/waretwin/frontend && npm run dev -- --host "${FRONTEND_HOST:-0.0.0.0}"
  cd $ROOT_DIR && source scripts/ros_env.sh && ros2 launch swerve_bringup system.launch.py
EOF
