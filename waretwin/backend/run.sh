#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
fi
source .venv/bin/activate
pip install -r requirements.txt
[ -f .env ] || cp .env.example .env
python manage.py migrate
python manage.py seed_demo
exec python manage.py runserver "${BACKEND_HOST:-0.0.0.0}:${BACKEND_PORT:-8000}"
