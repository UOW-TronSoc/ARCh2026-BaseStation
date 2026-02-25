from django.urls import path, re_path, include
from rest_framework.routers import DefaultRouter
from .views import *

router = DefaultRouter()
router.register(r'groups', ChecklistGroupViewSet)
router.register(r'tasks', ChecklistTaskViewSet)



urlpatterns = [
    path('status/', status_view, name='status'),
    path('checklist/', include(router.urls)),

    path('video_feed/<str:camera_name>/', get_frame, name='video_feed'),
    path('cameras/', get_camera_list, name='camera_list'),
    path('link-latency/', link_latency, name='link_latency'),

    # path('drivetrain-feedback/', get_drivetrain_feedback, name='drivetrain-feedback'),
    # path('core-feedback/', get_core_feedback, name='core-feedback'),

    # path('science-feedback/', get_science_feedback, name='science-feedback'),
    # path('science-control/', set_science_control, name='science-control'),

    # path('logs/', get_rover_logs, name='get_rover_logs'),

    # arm
    path('arm-feedback/', get_arm_feedback, name='arm_feedback'),
    path('arm-command/', send_arm_command, name='arm_command'),
    path("arm-velocity-command/", send_arm_velocity, name='arm_velocity'),
    
    
    # radio
    # path("radio-feedback/", get_radio_feedback, name="radio_feedback"),

    # battery
    path("battery-feedback/", battery_feedback_view),

    
    # logs
    path("list-logs/", list_logs),
    path("get-log/<str:filename>/", get_log_file),
    path("django-logs/", get_django_logs),
]