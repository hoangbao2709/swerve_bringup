#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
fi
source .venv/bin/activate
[ -f .env ] || cp .env.example .env
# A direct ./run.sh invocation should be safe on a fresh checkout too.  Replace
# template secrets once, but never overwrite an operator-provided value.
if grep -qE '^WARETWIN_ROS_BRIDGE_TOKEN=(change-me|)$' .env; then
  _bridge_token="$(python -c 'import secrets; print(secrets.token_urlsafe(32))')"
  sed -i "s|^WARETWIN_ROS_BRIDGE_TOKEN=.*|WARETWIN_ROS_BRIDGE_TOKEN=$_bridge_token|" .env
fi
if grep -qE '^DJANGO_SECRET_KEY=change-me-in-production$' .env; then
  _django_secret="$(python -c 'import secrets; print(secrets.token_urlsafe(48))')"
  sed -i "s|^DJANGO_SECRET_KEY=.*|DJANGO_SECRET_KEY=$_django_secret|" .env
fi
# Install only when the runtime dependencies are not present. The full setup
# script owns deterministic dependency installation; run.sh stays cheap and
# safe to call repeatedly during development.
if ! python -c 'import django, channels, daphne, dotenv, pydantic' >/dev/null 2>&1; then
  pip install -r requirements.txt
fi
# Load local dotenv values while preserving variables explicitly supplied by a
# service manager/start_stack.sh (notably the selected fallback port).
declare -A _explicit_env=()
for _key in DJANGO_DEBUG DJANGO_ALLOWED_HOSTS CORS_ALLOWED_ORIGINS BACKEND_HOST BACKEND_PORT FRONTEND_HOST FRONTEND_PORT ROS_DOMAIN_ID ROS_WS_URL WARETWIN_RUNTIME_MODE WARETWIN_ROS_BRIDGE_TOKEN WARETWIN_ARTIFACT_ROOT; do
  if [[ ${!_key+x} ]]; then _explicit_env["$_key"]="${!_key}"; fi
done
set -a
# shellcheck disable=SC1091
source .env
set +a
for _key in "${!_explicit_env[@]}"; do export "$_key=${_explicit_env[$_key]}"; done
unset _key _explicit_env
python manage.py migrate
python manage.py seed_demo
exec python manage.py runserver "${BACKEND_HOST:-0.0.0.0}:${BACKEND_PORT:-8000}"
