from django.urls import re_path
from .consumers import TwinConsumer
from .ros_bridge_consumer import RosBridgeConsumer

websocket_urlpatterns = [
    re_path(r'^ws/?$', TwinConsumer.as_asgi()),
    re_path(r'^ws/ros/?$', RosBridgeConsumer.as_asgi()),
]
