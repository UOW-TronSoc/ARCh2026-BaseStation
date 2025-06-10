from django.contrib import admin
from .models import ChecklistGroup, ChecklistTask

admin.site.register(ChecklistGroup)
admin.site.register(ChecklistTask)
