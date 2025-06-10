from django_redis import get_redis_connection
from django.db import models


class ChecklistGroup(models.Model):
    name = models.CharField(max_length=100)

    def __str__(self):
        return self.name

class ChecklistTask(models.Model):
    group = models.ForeignKey(ChecklistGroup, related_name='tasks', on_delete=models.CASCADE)
    text = models.TextField()
    completed = models.BooleanField(default=False)
    value = models.TextField(blank=True, null=True)
    parent = models.ForeignKey('self', null=True, blank=True, related_name='subtasks', on_delete=models.CASCADE)

    def __str__(self):
        return self.text
