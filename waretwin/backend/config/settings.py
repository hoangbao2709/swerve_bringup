from pathlib import Path
import ipaddress
import os
from urllib.parse import urlsplit

from dotenv import load_dotenv

BASE_DIR = Path(__file__).resolve().parent.parent
PROJECT_ROOT = BASE_DIR.parent.parent
load_dotenv(BASE_DIR / '.env')
_xdg_state_home = Path(os.getenv('XDG_STATE_HOME', Path.home() / '.local/state')).expanduser()
if not _xdg_state_home.is_absolute():
    _xdg_state_home = Path.home() / '.local/state'
RUNTIME_DIR = Path(os.getenv('WARETWIN_RUNTIME_DIR', _xdg_state_home / 'waretwin')).expanduser()
LOG_DIR = Path(os.getenv('WARETWIN_LOG_DIR', RUNTIME_DIR / 'logs')).expanduser()
LOG_DIR.mkdir(parents=True, exist_ok=True, mode=0o700)
os.chmod(LOG_DIR, 0o700)

# Load the backend-local environment before reading any setting.  override=False
# keeps explicit process/container environment variables authoritative.
SECRET_KEY = os.getenv('DJANGO_SECRET_KEY', 'development-only-change-me')
DEBUG = os.getenv('DJANGO_DEBUG', '0') == '1'
ALLOWED_HOSTS = [x.strip() for x in os.getenv('DJANGO_ALLOWED_HOSTS', 'localhost,127.0.0.1').split(',') if x.strip()]

INSTALLED_APPS = [
    # Kept only because historical twin models reference nullable auth.User
    # foreign keys; no application request or WebSocket uses Django users.
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.staticfiles',
    'channels',
    'twin',
]
MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware',
    'twin.middleware.SimpleCorsMiddleware',
    'django.middleware.common.CommonMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.middleware.clickjacking.XFrameOptionsMiddleware',
]
ROOT_URLCONF = 'config.urls'
TEMPLATES = [{
    'BACKEND': 'django.template.backends.django.DjangoTemplates',
    'DIRS': [],
    'APP_DIRS': True,
    'OPTIONS': {'context_processors': ['django.template.context_processors.request']},
}]
WSGI_APPLICATION = 'config.wsgi.application'
ASGI_APPLICATION = 'config.asgi.application'

DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': Path(os.getenv('WARETWIN_DATABASE_PATH', RUNTIME_DIR / 'db.sqlite3')),
        # SQLite is development/local storage here. Wait briefly for the single
        # writer instead of immediately returning OperationalError under ASGI.
        'OPTIONS': {'timeout': 30},
    }
}
_database_path = Path(os.getenv('WARETWIN_DATABASE_PATH', RUNTIME_DIR / 'db.sqlite3')).expanduser().resolve()
_backend_root = BASE_DIR.resolve()
if not DEBUG:
    try:
        _database_path.relative_to(_backend_root)
    except ValueError:
        pass
    else:
        raise RuntimeError('production runtime database must be outside the backend source/install directory')
DATABASES['default']['NAME'] = _database_path
LANGUAGE_CODE = 'en-us'
TIME_ZONE = 'UTC'
USE_I18N = True
USE_TZ = True
STATIC_URL = 'static/'
STATIC_ROOT = RUNTIME_DIR / 'static'
WARETWIN_LAYOUT_DIR = RUNTIME_DIR / 'layouts'
DEFAULT_AUTO_FIELD = 'django.db.models.BigAutoField'

CHANNEL_LAYERS = {
    'default': {
        'BACKEND': 'channels.layers.InMemoryChannelLayer'
    }
}
DATA_UPLOAD_MAX_MEMORY_SIZE = 12 * 1024 * 1024

LOGGING = {
    'version': 1,
    'disable_existing_loggers': False,
    'formatters': {
        'waretwin': {
            'format': '{asctime} {levelname} {name} {message}',
            'style': '{',
        },
    },
    'handlers': {
        'backend_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': str(LOG_DIR / 'backend.log'),
            'maxBytes': 5 * 1024 * 1024,
            'backupCount': 5,
            'encoding': 'utf-8',
            'formatter': 'waretwin',
        },
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'waretwin',
        },
    },
    'loggers': {
        'django': {'handlers': ['backend_file', 'console'], 'level': 'INFO', 'propagate': False},
        'django.server': {'handlers': ['backend_file', 'console'], 'level': 'INFO', 'propagate': False},
        'twin': {'handlers': ['backend_file', 'console'], 'level': 'INFO', 'propagate': False},
    },
}

WARETWIN_RUNTIME_MODES = ('LOCAL_SIM', 'GAZEBO_ROS', 'REAL_ROBOT')
WARETWIN_RUNTIME_MODE = os.getenv('WARETWIN_RUNTIME_MODE', 'GAZEBO_ROS').upper()
if WARETWIN_RUNTIME_MODE not in WARETWIN_RUNTIME_MODES:
    raise RuntimeError(
        f'Invalid WARETWIN_RUNTIME_MODE={WARETWIN_RUNTIME_MODE!r}; '
        f'expected one of: {", ".join(WARETWIN_RUNTIME_MODES)}'
    )
WARETWIN_REAL_RUNTIME_ADAPTER = os.getenv('WARETWIN_REAL_RUNTIME_ADAPTER', '').strip()
WARETWIN_STACK_RUNTIME_DIR = Path(os.getenv('WARETWIN_STACK_RUNTIME_DIR', PROJECT_ROOT / '.runtime'))
WARETWIN_ROS_BRIDGE_TOKEN = os.getenv('WARETWIN_ROS_BRIDGE_TOKEN', '')
WARETWIN_VDA5050_ENCRYPTION_KEY = os.getenv('WARETWIN_VDA5050_ENCRYPTION_KEY', '')
WARETWIN_ROS_HEARTBEAT_TIMEOUT_S = float(os.getenv('WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', '10.0'))
WARETWIN_VERSION = os.getenv('WARETWIN_VERSION', '0.1.0')
# Immutable publish artifacts live outside the source/config tree. Tests and
# deployments may override this with a warehouse-specific volume. Treat an
# empty env value as "use the project default" rather than Path('.') so a
# clean `.env.example` remains portable.
_artifact_root = os.getenv('WARETWIN_ARTIFACT_ROOT', '').strip()
WARETWIN_ARTIFACT_ROOT = Path(_artifact_root or (BASE_DIR.parent.parent / 'generated' / 'maps'))

# The browser HMI intentionally has no application login screen. LOCAL_LOOPBACK
# trusts only same-host users/processes and rejects cross-origin browser calls.
# PROTECTED_LAN requires a same-host trusted auth gateway that signs short-lived
# assertions; no operator secret is ever included in browser runtime-config.js.
WARETWIN_OPERATOR_AUTH_MODE = os.getenv('WARETWIN_OPERATOR_AUTH_MODE', 'LOCAL_LOOPBACK').strip().upper()
if WARETWIN_OPERATOR_AUTH_MODE not in ('LOCAL_LOOPBACK', 'PROTECTED_LAN'):
    raise RuntimeError('WARETWIN_OPERATOR_AUTH_MODE must be LOCAL_LOOPBACK or PROTECTED_LAN')

_cors_origins = [x.strip() for x in os.getenv(
    'CORS_ALLOWED_ORIGINS', 'http://localhost:5173,http://127.0.0.1:5173').split(',') if x.strip()]
_operator_origins_raw = os.getenv('WARETWIN_OPERATOR_ALLOWED_ORIGINS', '').strip()
if WARETWIN_OPERATOR_AUTH_MODE == 'PROTECTED_LAN':
    _operator_origins = [x.strip() for x in os.getenv(
        'WARETWIN_PUBLIC_ORIGINS', _operator_origins_raw).split(',') if x.strip()]
    WARETWIN_OPERATOR_GATEWAY_SECRET = os.getenv('WARETWIN_OPERATOR_GATEWAY_SECRET', '')
    if len(WARETWIN_OPERATOR_GATEWAY_SECRET.encode('utf-8')) < 32:
        raise RuntimeError('PROTECTED_LAN requires WARETWIN_OPERATOR_GATEWAY_SECRET of at least 32 bytes')
    if not _operator_origins:
        raise RuntimeError('PROTECTED_LAN requires WARETWIN_PUBLIC_ORIGINS')
    for _origin in _operator_origins:
        _parsed_origin = urlsplit(_origin)
        if (_parsed_origin.scheme != 'https' or not _parsed_origin.netloc
                or _parsed_origin.path or _parsed_origin.query or _parsed_origin.fragment
                or _parsed_origin.username or _parsed_origin.password):
            raise RuntimeError('PROTECTED_LAN public origins must be exact HTTPS origins without paths')
    _proxy_cidrs = [x.strip() for x in os.getenv('WARETWIN_TRUSTED_PROXY_CIDRS', '').split(',') if x.strip()]
    if not _proxy_cidrs:
        raise RuntimeError('PROTECTED_LAN requires WARETWIN_TRUSTED_PROXY_CIDRS')
    try:
        WARETWIN_TRUSTED_PROXY_NETWORKS = tuple(ipaddress.ip_network(value, strict=False) for value in _proxy_cidrs)
    except ValueError as exc:
        raise RuntimeError('WARETWIN_TRUSTED_PROXY_CIDRS contains an invalid network') from exc
    if any(not (network.network_address.is_loopback and network.broadcast_address.is_loopback)
           for network in WARETWIN_TRUSTED_PROXY_NETWORKS):
        raise RuntimeError('PROTECTED_LAN requires a same-host gateway; trusted proxy CIDRs must be loopback-only')
else:
    WARETWIN_OPERATOR_GATEWAY_SECRET = ''
    WARETWIN_TRUSTED_PROXY_NETWORKS = ()
    _operator_origins = [x.strip() for x in (_operator_origins_raw.split(',') if _operator_origins_raw else _cors_origins) if x.strip()]
    for _origin in _operator_origins:
        _parsed_origin = urlsplit(_origin)
        try:
            _origin_is_loopback = _parsed_origin.hostname == 'localhost' or bool(
                _parsed_origin.hostname and ipaddress.ip_address(_parsed_origin.hostname).is_loopback)
        except ValueError:
            _origin_is_loopback = False
        if (_parsed_origin.scheme != 'http' or not _parsed_origin.netloc or not _origin_is_loopback
                or _parsed_origin.path or _parsed_origin.query or _parsed_origin.fragment
                or _parsed_origin.username or _parsed_origin.password or _origin == '*'):
            raise RuntimeError('LOCAL_LOOPBACK allows only explicit HTTP origins on loopback hosts')
WARETWIN_OPERATOR_ALLOWED_ORIGINS = tuple(dict.fromkeys(_operator_origins))
CORS_ALLOWED_ORIGINS = WARETWIN_OPERATOR_ALLOWED_ORIGINS
if WARETWIN_OPERATOR_AUTH_MODE == 'PROTECTED_LAN':
    # Django validates Host before the signed gateway assertion is reached.
    # Admit only the exact configured HTTPS origin hostnames, never a wildcard.
    _public_hosts = []
    for _origin in WARETWIN_OPERATOR_ALLOWED_ORIGINS:
        _public_hosts.append(urlsplit(_origin).hostname)
        # Accessing .port forces urllib to reject malformed/out-of-range ports.
        _ = urlsplit(_origin).port
    ALLOWED_HOSTS = list(dict.fromkeys([*ALLOWED_HOSTS, *[host for host in _public_hosts if host]]))

print(f'WARETWIN runtime mode: {WARETWIN_RUNTIME_MODE}', flush=True)
