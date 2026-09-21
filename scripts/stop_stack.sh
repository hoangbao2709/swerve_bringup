#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/stack_common.sh"

for component in ros frontend backend; do
  if stack_owned_pid "$component" || stack_owned_group "$component"; then
    echo "Stopping $component"
    stack_kill_owned "$component"
  elif [[ -f "$(stack_pid_file "$component")" ]]; then
    echo "Removing stale $component pid file"
    stack_clear_pid "$component"
  else
    echo "$component is not owned by this stack"
  fi
done
rm -f "$STACK_RUNTIME_DIR/stack.env"
echo '[OK] Project processes stopped; unrelated processes were not touched.'
