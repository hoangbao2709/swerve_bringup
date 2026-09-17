from pathlib import Path
import os

from dotenv import load_dotenv

BASE_DIR = Path(__file__).resolve().parent.parent

# Load the backend-local environment before reading any setting.  override=False
# keeps explicit process/container environment variables authoritative.
load_dotenv(BASE_DIR / '.env')

SECRET_KEY = os.getenv('DJANGO_SECRET_KEY', 'development-only-change-me')
DEBUG = os.getenv('DJANGO_DEBUG', '1') == '1'
ALLOWED_HOSTS = [x.strip() for x in os.getenv('DJANGO_ALLOWED_HOSTS', '*').split(',') if x.strip()]

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

WARETWIN_RUNTIME_MODES = ('LOCAL_SIM', 'GAZEBO_ROS', 'REAL_ROBOT')
WARETWIN_RUNTIME_MODE = os.getenv('WARETWIN_RUNTIME_MODE', 'LOCAL_SIM').upper()
if WARETWIN_RUNTIME_MODE not in WARETWIN_RUNTIME_MODES:
    raise RuntimeError(
        f'Invalid WARETWIN_RUNTIME_MODE={WARETWIN_RUNTIME_MODE!r}; '
        f'expected one of: {", ".join(WARETWIN_RUNTIME_MODES)}'
    )
WARETWIN_ROS_BRIDGE_TOKEN = os.getenv('WARETWIN_ROS_BRIDGE_TOKEN', '')
WARETWIN_ROS_HEARTBEAT_TIMEOUT_S = float(os.getenv('WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', '3.0'))

print(f'WARETWIN runtime mode: {WARETWIN_RUNTIME_MODE}', flush=True)
