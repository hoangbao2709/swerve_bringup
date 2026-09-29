#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"

stop_failures=0
for component in ros frontend backend; do
  if stack_owned_pid "$component" || stack_owned_group "$component"; then
    pgid="$(stack_pgid "$component" 2>/dev/null || true)"
    echo "Stopping $component"
    stack_kill_owned "$component"
    if stack_group_has_live_process "$pgid"; then
      echo "[FAIL] $component process group $pgid still has live processes" >&2
      stop_failures=$((stop_failures + 1))
    else
      echo "[OK] $component process group stopped"
    fi
  elif [[ -f "$(stack_pid_file "$component")" ]]; then
    echo "Removing stale $component pid file"
    stack_clear_pid "$component"
  else
    echo "$component is not owned by this stack"
  fi
done
rm -f "$STACK_RUNTIME_DIR/stack.env"
if ((stop_failures)); then
  exit 1
fi
echo '[OK] Project processes stopped; unrelated processes were not touched.'
