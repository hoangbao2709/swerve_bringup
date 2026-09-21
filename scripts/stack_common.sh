#!/usr/bin/env bash

STACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STACK_RUNTIME_DIR="$STACK_ROOT/.runtime"
STACK_LOG_DIR="$STACK_ROOT/logs"
mkdir -p "$STACK_RUNTIME_DIR" "$STACK_LOG_DIR"

stack_pid_file() { printf '%s/%s.pid' "$STACK_RUNTIME_DIR" "$1"; }
stack_log_file() { printf '%s/%s.log' "$STACK_LOG_DIR" "$1"; }

stack_pid() {
  local name="$1" file
  file="$(stack_pid_file "$name")"
  [[ -f "$file" ]] || return 1
  local pid
  pid="$(sed -n '1p' "$file" 2>/dev/null || true)"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "$pid"
}

stack_pgid() {
  local name="$1" file pgid pid
  file="$(stack_pid_file "$name")"
  [[ -f "$file" ]] || return 1
  pgid="$(sed -n '2p' "$file" 2>/dev/null || true)"
  if [[ "$pgid" =~ ^[0-9]+$ ]]; then
    printf '%s\n' "$pgid"
    return 0
  fi
  pid="$(stack_pid "$name" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' '
}

stack_cmdline() {
  local pid="$1"
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$pid/cmdline"
}

stack_owned_pid() {
  local name="$1" pid cmd cwd
  pid="$(stack_pid "$name" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  cmd="$(stack_cmdline "$pid" 2>/dev/null || true)"
  cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
  case "$name" in
    backend) [[ "$cwd" == "$STACK_ROOT/waretwin/backend" ]] && [[ "$cmd" == *"manage.py runserver"* || "$cmd" == *"$STACK_ROOT/waretwin/backend/run.sh"* ]] ;;
    frontend) [[ "$cwd" == "$STACK_ROOT/waretwin/frontend" ]] && [[ "$cmd" == *"vite"* || "$cmd" == *"npm"* ]] ;;
    ros) [[ "$cwd" == "$STACK_ROOT" ]] && [[ "$cmd" == *"system.launch.py"* || "$cmd" == *"ros2 launch swerve_bringup"* ]] ;;
    *) return 1 ;;
  esac
}

stack_write_pid() {
  local name="$1" pid="$2" pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  printf '%s\n%s\n' "$pid" "$pgid" > "$(stack_pid_file "$name")"
}
stack_clear_pid() { rm -f -- "$(stack_pid_file "$1")"; }

stack_owned_group() {
  local name="$1" pgid pid cmd cwd
  pgid="$(stack_pgid "$name" 2>/dev/null || true)"
  [[ "$pgid" =~ ^[0-9]+$ ]] || return 1
  while read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    [[ -r "/proc/$pid/cmdline" ]] || continue
    cmd="$(stack_cmdline "$pid" 2>/dev/null || true)"
    cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    case "$name" in
      backend) [[ "$cwd" == "$STACK_ROOT/waretwin/backend" ]] && [[ "$cmd" == *"manage.py"* || "$cmd" == *"run.sh"* ]] && return 0 ;;
      frontend) [[ "$cwd" == "$STACK_ROOT/waretwin/frontend" ]] && [[ "$cmd" == *"vite"* || "$cmd" == *"npm"* ]] && return 0 ;;
      ros) [[ "$cwd" == "$STACK_ROOT" || "$cmd" == *"$STACK_ROOT/install/"* ]] && [[ "$cmd" == *"gzserver"* || "$cmd" == *"ros2 launch swerve_bringup"* || "$cmd" == *"system.launch.py"* ]] && return 0 ;;
    esac
  done < <(ps -eo pid=,pgid= 2>/dev/null | awk -v group="$pgid" '$2 == group { print $1 }')
  return 1
}

stack_port_pid() {
  local port="$1"
  if command -v lsof >/dev/null 2>&1; then
    local pid
    pid="$(lsof -nP -t -iTCP:"$port" -sTCP:LISTEN 2>/dev/null | head -n1 || true)"
    [[ -n "$pid" ]] && { printf '%s\n' "$pid"; return 0; }
  fi
  ss -ltnp 2>/dev/null | awk -v p=":$port" '$4 ~ p"$" {print $NF}' | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | head -n1
}

stack_port_busy() {
  local port="$1"
  if command -v ss >/dev/null 2>&1 && ss -ltnH 2>/dev/null | awk -v p=":$port" '$4 ~ p"$" { found=1 } END { exit !found }'; then
    return 0
  fi
  [[ -n "$(stack_port_pid "$port")" ]]
}

stack_choose_port() {
  local requested="$1" first_fallback="$2" port
  for port in "$requested" $(seq "$first_fallback" $((first_fallback + 20))); do
    if ! stack_port_busy "$port"; then printf '%s\n' "$port"; return 0; fi
  done
  return 1
}

stack_port_owner_message() {
  local port="$1" pid
  pid="$(stack_port_pid "$port" || true)"
  if [[ -n "$pid" ]]; then
    printf 'port %s is occupied by PID %s: %s\n' "$port" "$pid" "$(ps -p "$pid" -o user=,args= 2>/dev/null || true)"
  else
    printf 'port %s is not available (owner could not be resolved)\n' "$port"
  fi
}

stack_kill_owned() {
  local name="$1" pid pgid
  if ! stack_owned_pid "$name" && ! stack_owned_group "$name"; then stack_clear_pid "$name"; return 0; fi
  pid="$(stack_pid "$name" 2>/dev/null || true)"
  pgid="$(stack_pgid "$name" 2>/dev/null || true)"
  [[ "$pgid" =~ ^[0-9]+$ ]] || { stack_clear_pid "$name"; return 0; }
  # The launch parent may have exited while project-owned children remain.
  # Once stack_owned_group() has validated the group, terminate that exact
  # group rather than relying on a possibly stale parent PID.
  kill -TERM -- "-$pgid" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 -- "-$pgid" 2>/dev/null || break; sleep 0.2; done
  if kill -0 -- "-$pgid" 2>/dev/null; then
    kill -KILL -- "-$pgid" 2>/dev/null || true
  fi
  stack_clear_pid "$name"
}

stack_wait_http() {
  local url="$1" attempts="${2:-30}"
  for _ in $(seq 1 "$attempts"); do
    if curl -fsS --max-time 1 "$url" >/dev/null 2>&1; then return 0; fi
    sleep 1
  done
  return 1
}
