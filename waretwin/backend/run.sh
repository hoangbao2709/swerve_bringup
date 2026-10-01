#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ]; then
  echo 'Missing backend/.venv. Run ../../scripts/setup_full_stack.sh before starting the backend.' >&2
  exit 1
fi
# A copied venv's activate script can still point at a different worktree.
# Select this checkout's interpreter explicitly for checks and the live server.
BACKEND_PYTHON_PATH="$PWD/.venv/bin/python"
[ -f .env ] || cp .env.example .env
clean_python() {
  env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH "$BACKEND_PYTHON_PATH" "$@"
}
# A direct ./run.sh invocation should be safe on a fresh checkout too.  Replace
# template secrets once, but never overwrite an operator-provided value.
if grep -qE '^WARETWIN_ROS_BRIDGE_TOKEN=(change-me|)$' .env; then
  _bridge_token="$(clean_python -c 'import secrets; print(secrets.token_urlsafe(32))')"
  sed -i "s|^WARETWIN_ROS_BRIDGE_TOKEN=.*|WARETWIN_ROS_BRIDGE_TOKEN=$_bridge_token|" .env
fi
if grep -qE '^DJANGO_SECRET_KEY=change-me-in-production$' .env; then
  _django_secret="$(clean_python -c 'import secrets; print(secrets.token_urlsafe(48))')"
  sed -i "s|^DJANGO_SECRET_KEY=.*|DJANGO_SECRET_KEY=$_django_secret|" .env
fi
# The full setup script owns deterministic dependency installation. Runtime
# startup must not silently mutate the Python environment; opt into the old
# developer convenience explicitly when needed.
if ! clean_python -c 'import django, channels, daphne, dotenv, pydantic, wsaccel' >/dev/null 2>&1; then
  if [[ "${WARETWIN_DEV_AUTO_INSTALL:-0}" == '1' ]]; then
    echo '[WARN] WARETWIN_DEV_AUTO_INSTALL=1: installing backend dependencies for development.' >&2
    clean_python -m pip install -r requirements.txt
  else
    echo 'Backend dependencies are missing. Run ../../scripts/setup_full_stack.sh, or set WARETWIN_DEV_AUTO_INSTALL=1 for development.' >&2
    exit 1
  fi
fi
# Load local dotenv values while preserving variables explicitly supplied by a
# service manager/start_stack.sh (notably the selected fallback port).
declare -A _explicit_env=()
for _key in DJANGO_DEBUG DJANGO_ALLOWED_HOSTS CORS_ALLOWED_ORIGINS BACKEND_HOST BACKEND_PORT FRONTEND_HOST FRONTEND_PORT ROS_DOMAIN_ID ROS_WS_URL WARETWIN_RUNTIME_MODE WARETWIN_ROS_BRIDGE_TOKEN WARETWIN_ARTIFACT_ROOT WARETWIN_DEV_AUTO_INSTALL TWIN_ADMIN_USERNAME TWIN_ADMIN_EMAIL TWIN_ADMIN_PASSWORD; do
  if [[ ${!_key+x} ]]; then _explicit_env["$_key"]="${!_key}"; fi
done
set -a
# shellcheck disable=SC1091
source .env
set +a
for _key in "${!_explicit_env[@]}"; do export "$_key=${_explicit_env[$_key]}"; done
unset _key _explicit_env
clean_python manage.py migrate
clean_python manage.py seed_demo
unset TWIN_ADMIN_PASSWORD
clean_python manage.py sync_master_data
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH "$BACKEND_PYTHON_PATH" manage.py runserver "${BACKEND_HOST:-0.0.0.0}:${BACKEND_PORT:-8000}"
