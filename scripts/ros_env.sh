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

# Do not let a Snap-hosted desktop (for example VS Code) inject its GTK
# libraries into native ROS GUI tools.  The Snap core20 libpthread is not
# ABI-compatible with Ubuntu 22.04 and otherwise makes RViz fail at startup.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
source /opt/ros/humble/setup.bash

if [[ ! -f "$ROOT_DIR/install/local_setup.bash" ]]; then
  echo "Workspace is not built. Run: $ROOT_DIR/scripts/build_ros.sh" >&2
  return 1
fi
source "$ROOT_DIR/install/local_setup.bash"

# Reuse the exact backend token and artifact root when the backend has been
# configured locally. The dotenv file contains shell-compatible KEY=VALUE lines.
if [[ -f "$ROOT_DIR/waretwin/backend/.env" ]]; then
  # Keep explicit service/launch environment authoritative, just like
  # backend/run.sh. This matters when start_stack selects fallback ports.
  _ros_env_domain_set=0; _ros_env_ws_set=0; _ros_env_artifact_set=0
  _ros_env_token_set=0; _ros_env_mode_set=0
  if [[ ${ROS_DOMAIN_ID+x} ]]; then _ros_env_domain_set=1; _ros_env_domain_saved="$ROS_DOMAIN_ID"; fi
  if [[ ${ROS_WS_URL+x} ]]; then _ros_env_ws_set=1; _ros_env_ws_saved="$ROS_WS_URL"; fi
  if [[ ${WARETWIN_ARTIFACT_ROOT+x} ]]; then _ros_env_artifact_set=1; _ros_env_artifact_saved="$WARETWIN_ARTIFACT_ROOT"; fi
  if [[ ${WARETWIN_ROS_BRIDGE_TOKEN+x} ]]; then _ros_env_token_set=1; _ros_env_token_saved="$WARETWIN_ROS_BRIDGE_TOKEN"; fi
  if [[ ${WARETWIN_RUNTIME_MODE+x} ]]; then _ros_env_mode_set=1; _ros_env_mode_saved="$WARETWIN_RUNTIME_MODE"; fi
  set -a
  # shellcheck disable=SC1090
  source "$ROOT_DIR/waretwin/backend/.env"
  set +a
  if (( _ros_env_domain_set )); then export ROS_DOMAIN_ID="$_ros_env_domain_saved"; fi
  if (( _ros_env_ws_set )); then export ROS_WS_URL="$_ros_env_ws_saved"; fi
  if (( _ros_env_artifact_set )); then export WARETWIN_ARTIFACT_ROOT="$_ros_env_artifact_saved"; fi
  if (( _ros_env_token_set )); then export WARETWIN_ROS_BRIDGE_TOKEN="$_ros_env_token_saved"; fi
  if (( _ros_env_mode_set )); then export WARETWIN_RUNTIME_MODE="$_ros_env_mode_saved"; fi
  unset _ros_env_domain_set _ros_env_ws_set _ros_env_artifact_set _ros_env_token_set _ros_env_mode_set
  unset _ros_env_domain_saved _ros_env_ws_saved _ros_env_artifact_saved _ros_env_token_saved _ros_env_mode_saved
fi
export WARETWIN_ARTIFACT_ROOT="${WARETWIN_ARTIFACT_ROOT:-$ROOT_DIR/generated/maps}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_WS_URL="${ROS_WS_URL:-ws://127.0.0.1:${BACKEND_PORT:-8000}/ws/ros}"

echo "ROS 2 Humble + swerve_bringup loaded"
echo "  swerve_bringup: $(ros2 pkg prefix swerve_bringup 2>/dev/null || echo missing)"
echo "  swerve_bridge:  $(ros2 pkg prefix swerve_bridge 2>/dev/null || echo missing)"
