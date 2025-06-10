from django.urls import path, re_path, include
from rest_framework.routers import DefaultRouter
from .views import *

router = DefaultRouter()
router.register(r'groups', ChecklistGroupViewSet)
router.register(r'tasks', ChecklistTaskViewSet)



urlpatterns = [
    path('checklist/', include(router.urls)),
    
    
    path('video_feed/<int:camera_id>/', get_frame, name='video_feed'),

    path('drivetrain-feedback/', get_drivetrain_feedback, name='drivetrain-feedback'),
    path('core-feedback/', get_core_feedback, name='core-feedback'),


    path('science-feedback/', get_science_feedback, name='science-feedback'),
    path('science-control/', set_science_control, name='science-control'),

    path('logs/', get_rover_logs, name='get_rover_logs'),
    
    # arm
    path('arm-feedback/', get_arm_feedback, name='arm_feedback'),
    path('arm-command/', send_arm_command, name='arm_command'),
    path("arm-velocity-command/", send_arm_velocity, name='arm_velocity'),
    
    
    # radio
    path("radio-feedback/", get_radio_feedback, name="radio_feedback"),
    
    # battery
    path('battery-feedback/', get_battery_feedback, name='battery-feedback'),
    
    #logs
    path("list-logs/", list_logs),
    path("get-log/<str:filename>/", get_log_file),


]