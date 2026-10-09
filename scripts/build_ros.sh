#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is required; install python3-colcon-common-extensions" >&2
  exit 1
fi

# The interactive shell on this machine may preload another ROS workspace.
# Build against a clean Humble underlay so CMake does not cache the wrong
# Python interpreter or Nav2 overlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH AMENT_TRACE_SETUP_FILES COLCON_TRACE
export AMENT_TRACE_SETUP_FILES=""
set +u
source /opt/ros/humble/setup.bash
set -u

# The bridge is intentionally kept under swerve_bridge/. Colcon stops normal
# discovery at the top-level swerve_bringup package, so both base paths are
# explicit here, including the nested web package.
colcon build \
  --cmake-clean-cache \
  --base-paths . swerve_bridge waretwin \
  --packages-select swerve_bringup swerve_bridge waretwin_web \
  --event-handlers console_direct+ "$@"

echo
echo "Build complete. In a ROS terminal run: source $ROOT_DIR/scripts/ros_env.sh"
