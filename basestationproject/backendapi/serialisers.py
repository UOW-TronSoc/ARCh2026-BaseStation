from rest_framework import serializers
from .models import ChecklistTask, ChecklistGroup

        
class ChecklistTaskSerializer(serializers.ModelSerializer):
    subtasks = serializers.SerializerMethodField()

    class Meta:
        model = ChecklistTask
        fields = ['id', 'group', 'text', 'completed', 'value', 'parent', 'subtasks']

    def get_subtasks(self, obj):
        return ChecklistTaskSerializer(obj.subtasks.all(), many=True).data


class ChecklistGroupSerializer(serializers.ModelSerializer):
    tasks = ChecklistTaskSerializer(many=True, read_only=True)

    class Meta:
        model = ChecklistGroup
        fields = ['id', 'name', 'tasks']
