"""Remove the stored PIN hash (e.g. to reset or before running set_pin again)."""
import os
from django.core.management.base import BaseCommand
from django.core.cache import cache
from backendapi.pin_auth import CACHE_KEY_PIN_HASH, PIN_FILE


class Command(BaseCommand):
    help = "Clear the stored PIN hash so a new PIN can be set"

    def handle(self, *args, **options):
        try:
            cache.delete(CACHE_KEY_PIN_HASH)
        except Exception:
            pass
        if os.path.isfile(PIN_FILE):
            os.remove(PIN_FILE)
        self.stdout.write(self.style.SUCCESS("PIN cleared. Run 'python manage.py set_pin 123456' to set a new one."))
