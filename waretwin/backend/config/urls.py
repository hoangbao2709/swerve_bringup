from django.urls import include, path
from twin.views import root

urlpatterns = [
    path('', root),
    path('api/', include('twin.urls')),
]
