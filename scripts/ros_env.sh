#!/usr/bin/env bash
# Source this file in every ROS terminal used by this project.
# It deliberately removes unrelated ROS overlays (for example cartoros2)
# before loading Humble and this workspace.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source scripts/ros_env.sh" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_ros_env_restore_nounset=0
[[ "$-" == *u* ]] && _ros_env_restore_nounset=1

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found at /opt/ros/humble" >&2
  unset _ros_env_restore_nounset
  return 1
fi

_ros_env_restore_shell_options() {
  local restore_nounset="${_ros_env_restore_nounset:-0}"
  unset -f _ros_env_restore_shell_options
  unset _ros_env_restore_nounset
  if (( restore_nounset )); then set -u; fi
}

# Do not let a Snap-hosted desktop (for example VS Code) inject its GTK
# libraries into native ROS GUI tools.  The Snap core20 libpthread is not
# ABI-compatible with Ubuntu 22.04 and otherwise makes RViz fail at startup.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
if ! source /opt/ros/humble/setup.bash; then
  echo "Failed to source ROS 2 Humble at /opt/ros/humble/setup.bash" >&2
  _ros_env_restore_shell_options
  return 1
fi

if [[ ! -f "$ROOT_DIR/install/local_setup.bash" ]]; then
  echo "Workspace is not built. Run: $ROOT_DIR/scripts/build_ros.sh" >&2
  _ros_env_restore_shell_options
  return 1
fi
if ! source "$ROOT_DIR/install/local_setup.bash"; then
  echo "Failed to source workspace overlay at $ROOT_DIR/install/local_setup.bash" >&2
  _ros_env_restore_shell_options
  return 1
fi

# Use the same runtime ownership check as status/smoke. A stack.env file can
# survive a launch crash, so its domain is trusted only while this checkout's
# stack still owns a live process.
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"
_ros_env_runtime_active=0
if [[ -f "$STACK_RUNTIME_DIR/stack.env" ]] && stack_runtime_active; then
  _ros_env_runtime_active=1
fi

# Reuse the exact backend token and artifact root when the backend has been
# configured locally. The dotenv file contains shell-compatible KEY=VALUE lines.
_ros_env_domain_set=0; _ros_env_ws_set=0; _ros_env_artifact_set=0
_ros_env_token_set=0; _ros_env_mode_set=0; _ros_env_rmw_set=0
_ros_env_localhost_set=0; _ros_env_fastdds_set=0
if [[ ${ROS_DOMAIN_ID+x} ]]; then _ros_env_domain_set=1; _ros_env_domain_saved="$ROS_DOMAIN_ID"; fi
if [[ ${ROS_WS_URL+x} ]]; then _ros_env_ws_set=1; _ros_env_ws_saved="$ROS_WS_URL"; fi
if [[ ${WARETWIN_ARTIFACT_ROOT+x} ]]; then _ros_env_artifact_set=1; _ros_env_artifact_saved="$WARETWIN_ARTIFACT_ROOT"; fi
if [[ ${WARETWIN_ROS_BRIDGE_TOKEN+x} ]]; then _ros_env_token_set=1; _ros_env_token_saved="$WARETWIN_ROS_BRIDGE_TOKEN"; fi
if [[ ${WARETWIN_RUNTIME_MODE+x} ]]; then _ros_env_mode_set=1; _ros_env_mode_saved="$WARETWIN_RUNTIME_MODE"; fi
if [[ ${RMW_IMPLEMENTATION+x} ]]; then _ros_env_rmw_set=1; _ros_env_rmw_saved="$RMW_IMPLEMENTATION"; fi
if [[ ${ROS_LOCALHOST_ONLY+x} ]]; then _ros_env_localhost_set=1; _ros_env_localhost_saved="$ROS_LOCALHOST_ONLY"; fi
if [[ ${FASTDDS_BUILTIN_TRANSPORTS+x} ]]; then _ros_env_fastdds_set=1; _ros_env_fastdds_saved="$FASTDDS_BUILTIN_TRANSPORTS"; fi
_ros_env_domain_source='default'
if [[ -f "$ROOT_DIR/waretwin/backend/.env" ]]; then
  # Keep explicit service/launch environment authoritative, just like
  # backend/run.sh. This matters when start_stack selects fallback ports. The
  # ROS domain is restored below, after the active runtime snapshot is checked.
  set -a
  # shellcheck disable=SC1090
  if ! source "$ROOT_DIR/waretwin/backend/.env"; then
    set +a
    echo "Failed to load backend runtime settings from waretwin/backend/.env" >&2
    _ros_env_restore_shell_options
    return 1
  fi
  set +a
  if (( _ros_env_domain_set )); then
    export ROS_DOMAIN_ID="$_ros_env_domain_saved"
    _ros_env_domain_source='shell'
  elif [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
    _ros_env_domain_source='backend/.env'
  fi
  if (( _ros_env_ws_set )); then export ROS_WS_URL="$_ros_env_ws_saved"; fi
  if (( _ros_env_artifact_set )); then export WARETWIN_ARTIFACT_ROOT="$_ros_env_artifact_saved"; fi
  if (( _ros_env_token_set )); then export WARETWIN_ROS_BRIDGE_TOKEN="$_ros_env_token_saved"; fi
  if (( _ros_env_mode_set )); then export WARETWIN_RUNTIME_MODE="$_ros_env_mode_saved"; fi
  if (( _ros_env_rmw_set )); then export RMW_IMPLEMENTATION="$_ros_env_rmw_saved"; fi
  if (( _ros_env_localhost_set )); then export ROS_LOCALHOST_ONLY="$_ros_env_localhost_saved"; fi
  if (( _ros_env_fastdds_set )); then export FASTDDS_BUILTIN_TRANSPORTS="$_ros_env_fastdds_saved"; fi
fi

if (( _ros_env_runtime_active )); then
  # An active launch snapshot owns all DDS settings used by the running stack.
  _ros_env_runtime_values="$({
    set -a
    # shellcheck disable=SC1090
    source "$STACK_RUNTIME_DIR/stack.env"
    printf '%s\n%s\n%s\n%s' "${ROS_DOMAIN_ID:-}" \
      "${RMW_IMPLEMENTATION:-}" "${ROS_LOCALHOST_ONLY:-}" \
      "${FASTDDS_BUILTIN_TRANSPORTS:-}"
  } 2>/dev/null)"
  mapfile -t _ros_env_runtime_parts <<<"$_ros_env_runtime_values"
  _ros_env_runtime_domain="${_ros_env_runtime_parts[0]:-}"
  if stack_valid_ros_domain "$_ros_env_runtime_domain"; then
    export ROS_DOMAIN_ID="$_ros_env_runtime_domain"
    _ros_env_domain_source='runtime stack.env'
  else
    echo "Warning: active runtime stack.env has invalid ROS_DOMAIN_ID='$_ros_env_runtime_domain'; keeping fallback" >&2
  fi
  [[ -n "${_ros_env_runtime_parts[1]:-}" ]] && export RMW_IMPLEMENTATION="${_ros_env_runtime_parts[1]}"
  [[ -n "${_ros_env_runtime_parts[2]:-}" ]] && export ROS_LOCALHOST_ONLY="${_ros_env_runtime_parts[2]}"
  if [[ -n "${_ros_env_runtime_parts[3]:-}" ]]; then
    export FASTDDS_BUILTIN_TRANSPORTS="${_ros_env_runtime_parts[3]}"
  else
    unset FASTDDS_BUILTIN_TRANSPORTS
  fi
fi

export WARETWIN_ARTIFACT_ROOT="${WARETWIN_ARTIFACT_ROOT:-$ROOT_DIR/generated/maps}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
# This VMware guest logged Fast DDS RTPS_TRANSPORT_SHM "Failed init_port"
# / "open_and_lock_file failed" errors when using the default transport.
# Keep UDPv4 consistent for every stack process and manually sourced ROS CLI.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_WS_URL="${ROS_WS_URL:-ws://127.0.0.1:${BACKEND_PORT:-8000}/ws/ros}"
if ! stack_valid_ros_domain "$ROS_DOMAIN_ID"; then
  echo "Invalid ROS_DOMAIN_ID=$ROS_DOMAIN_ID; use an integer from 0 to 232" >&2
  _ros_env_restore_shell_options
  return 1
fi
if [[ "$ROS_LOCALHOST_ONLY" != 0 && "$ROS_LOCALHOST_ONLY" != 1 ]]; then
  echo "Invalid ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY; use 0 or 1" >&2
  _ros_env_restore_shell_options
  return 1
fi

unset _ros_env_runtime_active _ros_env_runtime_domain
unset _ros_env_domain_set _ros_env_ws_set _ros_env_artifact_set _ros_env_token_set _ros_env_mode_set
unset _ros_env_domain_saved _ros_env_ws_saved _ros_env_artifact_saved _ros_env_token_saved _ros_env_mode_saved
unset _ros_env_rmw_set _ros_env_localhost_set _ros_env_fastdds_set
unset _ros_env_rmw_saved _ros_env_localhost_saved _ros_env_fastdds_saved
unset _ros_env_runtime_values _ros_env_runtime_parts
_ros_env_restore_shell_options

echo "ROS 2 Humble + swerve_bringup loaded"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID} (${_ros_env_domain_source})"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
echo "  FASTDDS_BUILTIN_TRANSPORTS: ${FASTDDS_BUILTIN_TRANSPORTS:-<unset>}"
echo "  swerve_bringup: $(ros2 pkg prefix swerve_bringup 2>/dev/null || echo missing)"
echo "  swerve_bridge:  $(ros2 pkg prefix swerve_bridge 2>/dev/null || echo missing)"
