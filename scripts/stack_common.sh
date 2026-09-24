#!/usr/bin/env bash

STACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STACK_RUNTIME_DIR="$STACK_ROOT/.runtime"
STACK_LOG_DIR="$STACK_ROOT/logs"
mkdir -p "$STACK_RUNTIME_DIR" "$STACK_LOG_DIR"

stack_pid_file() { printf '%s/%s.pid' "$STACK_RUNTIME_DIR" "$1"; }
stack_log_file() { printf '%s/%s.log' "$STACK_LOG_DIR" "$1"; }

stack_valid_ros_domain() {
  local domain="${1:-}"
  [[ "$domain" =~ ^[0-9]+$ ]] || return 1
  (( domain >= 0 && domain <= 232 ))
}

stack_controller_active() {
  local controller_output="${1:-}" wanted="${2:-}"
  printf '%s\n' "$controller_output" |
    sed $'s/\033\[[0-9;]*[[:alpha:]]//g' |
    awk -v wanted="$wanted" '$1 == wanted || index($1, wanted "[") == 1 { for (i = 2; i <= NF; i++) if (tolower($i) ~ /^active([[:space:]]|$)/) found = 1 } END { exit !found }'
}

stack_csv_add() {
  local current="${1:-}" value="${2:-}"
  if [[ -z "$value" ]]; then
    printf '%s\n' "$current"
  elif [[ -z "$current" ]]; then
    printf '%s\n' "$value"
  elif [[ ",$current," == *",$value,"* ]]; then
    printf '%s\n' "$current"
  else
    printf '%s,%s\n' "$current" "$value"
  fi
}

stack_host_ipv4s() {
  {
    if command -v ip >/dev/null 2>&1; then
      ip -4 -o addr show scope global 2>/dev/null | awk '{split($4, address, "/"); print address[1]}'
    fi
    hostname -I 2>/dev/null || true
  } | awk '
    function valid(ip, octet, count, i) {
      count = split(ip, octet, ".")
      if (count != 4) return 0
      for (i = 1; i <= 4; i++) {
        if (octet[i] !~ /^[0-9]+$/ || octet[i] < 0 || octet[i] > 255) return 0
      }
      return 1
    }
    {
      for (i = 1; i <= NF; i++) {
        ip = $i
        sub(/\/.*/, "", ip)
        if (valid(ip)) print ip
      }
    }
  ' | sort -u
}

stack_host_name() {
  local host
  host="$(hostname 2>/dev/null | tr -d '[:space:]' || true)"
  if [[ "$host" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ && "$host" != *..* ]]; then
    printf '%s\n' "$host"
  fi
}

stack_allowed_hosts() {
  local result="${1:-}" ip host
  result="$(stack_csv_add "$result" 'localhost')"
  result="$(stack_csv_add "$result" '127.0.0.1')"
  while read -r ip; do
    [[ -n "$ip" ]] || continue
    result="$(stack_csv_add "$result" "$ip")"
  done < <(stack_host_ipv4s)
  host="$(stack_host_name || true)"
  [[ -n "$host" ]] && result="$(stack_csv_add "$result" "$host")"
  printf '%s\n' "$result"
}

stack_cors_origins() {
  local port="$1" result="${2:-}" ip host
  result="$(stack_csv_add "$result" "http://localhost:$port")"
  result="$(stack_csv_add "$result" "http://127.0.0.1:$port")"
  while read -r ip; do
    [[ -n "$ip" ]] || continue
    result="$(stack_csv_add "$result" "http://$ip:$port")"
  done < <(stack_host_ipv4s)
  host="$(stack_host_name || true)"
  [[ -n "$host" ]] && result="$(stack_csv_add "$result" "http://$host:$port")"
  printf '%s\n' "$result"
}

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
    ros) [[ "$cwd" == "$STACK_ROOT" ]] && [[ "$cmd" == *"ros_stack_supervisor.py"* || "$cmd" == *"system.launch.py"* || "$cmd" == *"ros2 launch swerve_bringup"* ]] ;;
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
      ros) [[ "$cwd" == "$STACK_ROOT" || "$cmd" == *"$STACK_ROOT/install/"* ]] && [[ "$cmd" == *"ros_stack_supervisor.py"* || "$cmd" == *"gzserver"* || "$cmd" == *"ros2 launch swerve_bringup"* || "$cmd" == *"system.launch.py"* ]] && return 0 ;;
    esac
  done < <(ps -eo pid=,pgid= 2>/dev/null | awk -v group="$pgid" '$2 == group { print $1 }')
  return 1
}

stack_runtime_active() {
  # stack.env is a launch snapshot, not proof that the processes still exist.
  # Treat it as authoritative only while at least one owned project process or
  # process group is alive. This prevents a reset/crash from pinning status and
  # smoke probes to a stale DDS domain or fallback port.
  local component
  for component in backend frontend ros; do
    if stack_owned_pid "$component" || stack_owned_group "$component"; then
      return 0
    fi
  done
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

# A finite data probe is deliberately shared by status/smoke/readiness callers.
# `ros2 topic list` only proves graph discovery; the sensor_data-compatible
# subscriber below proves that at least one sample can actually be received.
stack_ros_topic_message() {
  local topic="$1" timeout_s="${2:-5}"
  local qos_args=(--qos-profile sensor_data --qos-reliability best_effort --qos-durability volatile)
  # /map is a latched OccupancyGrid in both Nav2 and the SLAM contract. Match
  # its reliable/transient-local profile so a late readiness probe receives
  # the existing map instead of waiting forever for a new publish.
  if [[ "$topic" == /map ]]; then
    qos_args=(--qos-reliability reliable --qos-durability transient_local)
  fi
  local deadline=$((SECONDS + timeout_s)) remaining attempt
  # Discovery can lose the first short-lived CLI subscriber on a busy Gazebo
  # host. Retry finite probes within the same overall deadline; this is still a
  # real message check and never falls back to topic-list membership.
  while (( SECONDS < deadline )); do
    remaining=$((deadline - SECONDS))
    attempt=$remaining
    (( attempt > 4 )) && attempt=4
    timeout "$attempt" ros2 topic echo --once \
      --no-daemon --spin-time 2 \
      "${qos_args[@]}" \
      "$topic" >/dev/null 2>&1 && return 0
  done
  return 1
}

stack_ros_data_probe() {
  local timeout_s="${1:-15}"
  shift
  # The probe owns its finite deadline; the outer guard catches a broken ROS
  # context without turning normal retries into an unbounded command.
  timeout "$((timeout_s + 5))" python3 "$STACK_ROOT/scripts/ros_data_probe.py" \
    --timeout "$timeout_s" "$@"
}

stack_ros_lifecycle_probe() {
  local timeout_s="${1:-20}"
  shift
  timeout "$((timeout_s + 5))" python3 "$STACK_ROOT/scripts/lifecycle_probe.py" \
    --timeout "$timeout_s" "$@"
}

stack_ros_tf_resolves() {
  local target="$1" source="$2" timeout_s="${3:-10}" output
  # Keep one tf2_echo process alive for the complete deadline. It retains its
  # TF buffer and naturally retries Invalid frame ID/extrapolation warnings;
  # restarting it every second would discard the buffer just as startup data
  # arrives.
  output="$(timeout "$timeout_s" ros2 run tf2_ros tf2_echo "$target" "$source" 2>&1 || true)"
  printf '%s\n' "$output" | rg -q 'Translation:|At time'
}

stack_ros_tf_probe() {
  local timeout_s="${1:-15}"
  shift
  timeout "$((timeout_s + 5))" python3 "$STACK_ROOT/scripts/tf_probe.py" \
    --timeout "$timeout_s" "$@"
}

stack_ros_lifecycle_active() {
  local node="$1" timeout_s="${2:-5}" deadline remaining probe output
  deadline=$((SECONDS + timeout_s))
  # A lifecycle service can miss one response while Gazebo is compiling or
  # servicing a sensor callback. Retry the same finite query until the caller's
  # deadline instead of treating one DDS round-trip as a state transition.
  while (( SECONDS < deadline )); do
    remaining=$((deadline - SECONDS))
    probe=$((remaining < 2 ? remaining : 2))
    (( probe < 1 )) && probe=1
    output="$(timeout "$probe" ros2 lifecycle get "/${node#/}" 2>/dev/null || true)"
    if printf '%s\n' "$output" | rg -qi '(^|[[:space:]])active([[:space:]]|$)'; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}
