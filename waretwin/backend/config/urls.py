from django.contrib import admin
from django.urls import include, path
from twin.views import root

urlpatterns = [
    path('django-admin/', admin.site.urls),
    path('', root),
    path('api/', include('twin.urls')),
]
