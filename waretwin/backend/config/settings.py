from pathlib import Path
import os

from dotenv import load_dotenv

BASE_DIR = Path(__file__).resolve().parent.parent
PROJECT_ROOT = BASE_DIR.parent.parent
LOG_DIR = PROJECT_ROOT / 'logs'
LOG_DIR.mkdir(parents=True, exist_ok=True)

# Load the backend-local environment before reading any setting.  override=False
# keeps explicit process/container environment variables authoritative.
load_dotenv(BASE_DIR / '.env')

SECRET_KEY = os.getenv('DJANGO_SECRET_KEY', 'development-only-change-me')
DEBUG = os.getenv('DJANGO_DEBUG', '0') == '1'
ALLOWED_HOSTS = [x.strip() for x in os.getenv('DJANGO_ALLOWED_HOSTS', 'localhost,127.0.0.1').split(',') if x.strip()]

INSTALLED_APPS = [
    'daphne',
    'django.contrib.admin',
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.messages',
    'django.contrib.staticfiles',
    'channels',
    'accounts',
    'twin',
]
MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware',
    'twin.middleware.SimpleCorsMiddleware',
    'django.contrib.sessions.middleware.SessionMiddleware',
    'django.middleware.common.CommonMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',
    'django.contrib.messages.middleware.MessageMiddleware',
    'django.middleware.clickjacking.XFrameOptionsMiddleware',
]
ROOT_URLCONF = 'config.urls'
TEMPLATES = [{
    'BACKEND': 'django.template.backends.django.DjangoTemplates',
    'DIRS': [],
    'APP_DIRS': True,
    'OPTIONS': {'context_processors': [
        'django.template.context_processors.request',
        'django.contrib.auth.context_processors.auth',
        'django.contrib.messages.context_processors.messages',
    ]},
}]
WSGI_APPLICATION = 'config.wsgi.application'
ASGI_APPLICATION = 'config.asgi.application'

DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': BASE_DIR / 'db.sqlite3',
        # SQLite is development/local storage here. Wait briefly for the single
        # writer instead of immediately returning OperationalError under ASGI.
        'OPTIONS': {'timeout': 30},
    }
}
AUTH_PASSWORD_VALIDATORS = []
LANGUAGE_CODE = 'en-us'
TIME_ZONE = 'UTC'
USE_I18N = True
USE_TZ = True
STATIC_URL = 'static/'
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

WARETWIN_RUNTIME_MODES = ('GAZEBO_ROS', 'REAL_ROBOT')
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

print(f'WARETWIN runtime mode: {WARETWIN_RUNTIME_MODE}', flush=True)
