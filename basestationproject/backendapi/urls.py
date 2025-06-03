from django.urls import path, include, re_path
from . import views

urlpatterns = [

    path('video_feed/<int:camera_id>/', views.get_frame, name='video_feed'),

    path('drivetrain-feedback/', views.get_drivetrain_feedback, name='drivetrain-feedback'),
    path('core-feedback/', views.get_core_feedback, name='core-feedback'),


    path('science-feedback/', views.get_science_feedback, name='science-feedback'),
    path('science-control/', views.set_science_control, name='science-control'),

    path('logs/', views.get_rover_logs, name='get_rover_logs'),
    
    # arm
    path('arm-feedback/', views.get_arm_feedback, name='arm_feedback'),
    path('arm-command/', views.send_arm_command, name='arm_command'),
    path("arm-velocity-command/", views.send_arm_velocity, name='arm_velocity'),
    
    
    # radio
    path("radio-feedback/", views.get_radio_feedback, name="radio_feedback"),
    
    # battery
    path('battery-feedback/', views.get_battery_feedback, name='battery-feedback'),
    
    #logs
    path("list-logs/", views.list_logs),
    path("get-log/<str:filename>/", views.get_log_file),


]