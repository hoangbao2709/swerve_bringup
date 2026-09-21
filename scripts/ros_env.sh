#!/usr/bin/env bash
# Source this file in every ROS terminal used by this project.
# It deliberately removes unrelated ROS overlays (for example cartoros2)
# before loading Humble and this workspace.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source scripts/ros_env.sh" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found at /opt/ros/humble" >&2
  return 1
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash

if [[ ! -f "$ROOT_DIR/install/local_setup.bash" ]]; then
  echo "Workspace is not built. Run: $ROOT_DIR/scripts/build_ros.sh" >&2
  return 1
fi
source "$ROOT_DIR/install/local_setup.bash"

# Reuse the exact backend token and artifact root when the backend has been
# configured locally. The dotenv file contains shell-compatible KEY=VALUE lines.
if [[ -f "$ROOT_DIR/waretwin/backend/.env" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ROOT_DIR/waretwin/backend/.env"
  set +a
fi
export WARETWIN_ARTIFACT_ROOT="${WARETWIN_ARTIFACT_ROOT:-$ROOT_DIR/generated/maps}"

echo "ROS 2 Humble + swerve_bringup loaded"
echo "  swerve_bringup: $(ros2 pkg prefix swerve_bringup 2>/dev/null || echo missing)"
echo "  swerve_bridge:  $(ros2 pkg prefix swerve_bridge 2>/dev/null || echo missing)"
