from django.contrib import admin
from .models import UserProfile, ApiToken
admin.site.register(UserProfile)
admin.site.register(ApiToken)
