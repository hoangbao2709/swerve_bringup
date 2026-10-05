from django.urls import path
from . import views, navigation_views, local_control_views

urlpatterns = [
    path('auth/login', views.auth_login),
    path('auth/logout', views.auth_logout),
    path('auth/me', views.auth_me),
    path('health', views.health),
    path('health/', views.health),
    path('system/status', views.system_status),
    path('system/status/', views.system_status),
    path('layout', views.layout_get),
    path('map/sync-status', views.map_sync_status_view),
    path('robots/<str:robot_id>/navigation-tags', navigation_views.robot_tags),
    path('robots/<str:robot_id>/emergency-stop', navigation_views.emergency_stop),
    path('robots/<str:robot_id>/clear-emergency-stop', navigation_views.clear_emergency_stop),
    path('robots/<str:robot_id>/local/maps', local_control_views.local_maps),
    path('robots/<str:robot_id>/local/runtime-mode', local_control_views.local_runtime_mode),
    path('robots/<str:robot_id>/local/mapping/<str:action>', local_control_views.mapping_command),
    path('robots/<str:robot_id>/local/maps/save', local_control_views.save_robot_map),
    path('robots/<str:robot_id>/local/maps/load', local_control_views.load_robot_map),
    path('robots/<str:robot_id>/local/maps/resume-session', local_control_views.resume_robot_slam_session),
    path('robots/<str:robot_id>/local/initial-pose', local_control_views.initialize_robot_pose),
    path('robots/<str:robot_id>/local/vda5050', local_control_views.vda5050_configuration),
    path('robots/<str:robot_id>/local/vda5050/test', local_control_views.vda5050_test_connection),
]
