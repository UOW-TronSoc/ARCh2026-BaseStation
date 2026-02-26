"""
PIN-based auth: hash stored in cache (Redis) or file fallback, verified against Django's password hasher.
PIN is never stored in plain text. After verify, session holds pin_verified flag.
"""
import os
from django.core.cache import cache
from django.conf import settings
from django.contrib.auth.hashers import make_password, check_password

CACHE_KEY_PIN_HASH = "basestation:pin_hash"
PIN_FILE = getattr(settings, "PIN_HASH_FILE", None) or os.path.join(settings.BASE_DIR, ".pin_hash")


def _read_file_hash():
    if os.path.isfile(PIN_FILE):
        try:
            with open(PIN_FILE, "r") as f:
                return f.read().strip() or None
        except OSError:
            pass
    return None


def _write_file_hash(hash_val: str) -> None:
    with open(PIN_FILE, "w") as f:
        f.write(hash_val)
    os.chmod(PIN_FILE, 0o600)


def _get_stored_hash():
    """Try cache first, then file fallback."""
    try:
        val = cache.get(CACHE_KEY_PIN_HASH)
        if val:
            return val
    except Exception:
        pass
    return _read_file_hash()


def _set_stored_hash(hash_val: str) -> None:
    """Write to cache and file (for persistence when Redis unavailable)."""
    try:
        cache.set(CACHE_KEY_PIN_HASH, hash_val, timeout=None)
    except Exception:
        pass
    _write_file_hash(hash_val)


def is_pin_configured():
    """Returns True if a PIN hash exists."""
    return _get_stored_hash() is not None


def set_pin_hash(pin: str) -> None:
    """Store PBKDF2 hash of PIN. Run via management command only."""
    if len(pin) != 6 or not pin.isdigit():
        raise ValueError("PIN must be exactly 6 digits")
    _set_stored_hash(make_password(pin, salt=None))


def verify_pin(pin: str) -> bool:
    """Verify PIN against stored hash. Returns True if correct."""
    stored = _get_stored_hash()
    if not stored:
        return False
    if len(pin) != 6 or not pin.isdigit():
        return False
    return check_password(pin, stored)
