$ErrorActionPreference = "Stop"

if (-not (Test-Path ".venv")) {
    python -m venv .venv
}

& .\.venv\Scripts\python.exe -m pip install -r requirements.txt
if (-not (Test-Path ".env")) {
    Copy-Item .env.example .env
}
& .\.venv\Scripts\python.exe manage.py migrate
& .\.venv\Scripts\python.exe manage.py seed_demo
& .\.venv\Scripts\python.exe manage.py sync_master_data
& .\.venv\Scripts\python.exe manage.py runserver 0.0.0.0:8000
