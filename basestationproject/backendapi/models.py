from django_redis import get_redis_connection
from django.db import models

class ExampleModel(models.Model):
    name = models.CharField(max_length=100)
    redis_connection = get_redis_connection()

    def save(self, *args, **kwargs):
        self.redis_connection.set(self.name, self.name)
        super().save(*args, **kwargs)

    def retrieve_data(self):
        return self.redis_connection.get(self.name)