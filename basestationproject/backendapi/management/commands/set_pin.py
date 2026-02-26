"""
Set the 6-digit PIN for basestation access. Run once during initial setup.
Usage: python manage.py set_pin 123456
The hash is stored in Redis cache. The plain PIN is never stored.
"""
from django.core.management.base import BaseCommand
from backendapi.pin_auth import set_pin_hash, is_pin_configured


class Command(BaseCommand):
    help = "Set the 6-digit basestation PIN (stored as PBKDF2 hash in cache)"

    def add_arguments(self, parser):
        parser.add_argument("pin", type=str, help="6-digit PIN (e.g. 123456)")

    def handle(self, *args, **options):
        pin = options["pin"].strip()
        if len(pin) != 6 or not pin.isdigit():
            self.stderr.write(self.style.ERROR("PIN must be exactly 6 digits"))
            return
        if is_pin_configured():
            self.stdout.write(self.style.WARNING("PIN already set. Use 'python manage.py clear_pin' first to reset."))
            return
        set_pin_hash(pin)
        self.stdout.write(self.style.SUCCESS("PIN set successfully. Hash stored in cache."))
