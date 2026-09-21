#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_APT_PACKAGES=(
  ros-humble-gazebo-ros-pkgs
  ros-humble-gazebo-ros2-control
  ros-humble-gazebo-plugins
  ros-humble-ros2-controllers
  ros-humble-robot-localization
  ros-humble-pointcloud-to-laserscan
  ros-humble-slam-toolbox
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-rviz2
  ros-humble-xacro
  ros-humble-joint-state-publisher-gui
  python3-colcon-common-extensions
  python3-rosdep
  python3-venv
  python3-yaml
  python3-websocket
)

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

echo "Installing ROS/system dependencies..."
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y "${ROS_APT_PACKAGES[@]}"

command -v node >/dev/null || { echo "Node.js 18+ is required" >&2; exit 1; }
command -v npm >/dev/null || { echo "npm is required" >&2; exit 1; }

cd "$ROOT_DIR/waretwin/backend"
[[ -x .venv/bin/python ]] || python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
[[ -f .env ]] || cp .env.example .env
.venv/bin/python manage.py migrate --noinput
.venv/bin/python manage.py seed_demo

cd "$ROOT_DIR/waretwin/frontend"
npm ci

cd "$ROOT_DIR"
scripts/build_ros.sh

cat <<EOF

Setup complete.

Start ROS with:
  source $ROOT_DIR/scripts/ros_env.sh
  ros2 launch swerve_bringup system.launch.py

Start backend with:
  cd $ROOT_DIR/waretwin/backend && ./run.sh

Start frontend with:
  cd $ROOT_DIR/waretwin/frontend && npm run dev -- --host 0.0.0.0
EOF
