from django.urls import path, re_path, include
from rest_framework.routers import DefaultRouter
from .views import *
from .pin_views import auth_status_view, pin_verify_view

router = DefaultRouter()
router.register(r'groups', ChecklistGroupViewSet)
router.register(r'tasks', ChecklistTaskViewSet)



urlpatterns = [
    path('status/', status_view, name='status'),
    path('auth-status/', auth_status_view, name='auth_status'),
    path('pin-verify/', pin_verify_view, name='pin_verify'),
    path('checklist/', include(router.urls)),

    path('video_feed/<str:camera_name>/', mjpeg_stream, name='video_feed'),
    path('cameras/', get_camera_list, name='camera_list'),
    path('camera-debug/', camera_debug, name='camera_debug'),
    path('link-latency/', link_latency, name='link_latency'),
    path('servo-demo/', run_servo_demo, name='servo_demo'),

    # path('drivetrain-feedback/', get_drivetrain_feedback, name='drivetrain-feedback'),
    # path('core-feedback/', get_core_feedback, name='core-feedback'),

    path('science-feedback/', get_science_feedback, name='science-feedback'),
    path('science-control/', set_science_control, name='science-control'),
    path('nir-servo-demo/', run_nir_servo_demo, name='nir_servo_demo'),

    # path('logs/', get_rover_logs, name='get_rover_logs'),

    # arm
    path('arm-feedback/', get_arm_feedback, name='arm_feedback'),
    path("arm-velocity-command/", send_arm_velocity, name='arm_velocity'),
    path("arm-ee-command/", send_arm_ee_command, name='arm_ee_command'),
    path("arm-mode/", set_arm_mode, name='arm_mode'),
    
    
    # radio
    # path("radio-feedback/", get_radio_feedback, name="radio_feedback"),

    # battery
    path("battery-feedback/", battery_feedback_view),

    
    # logs
    path("list-logs/", list_logs),
    path("get-log/<str:filename>/", get_log_file),
    path("django-logs/", get_django_logs),
]