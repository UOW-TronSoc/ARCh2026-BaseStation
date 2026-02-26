"""
Protects /api/* routes with PIN auth. Exempts pin-verify, auth-status, and status.
Run after SessionMiddleware. Returns 401 if not authenticated and path is protected.
"""
import logging
from django.http import JsonResponse

logger = logging.getLogger(__name__)
EXEMPT_PREFIXES = ("/api/pin-verify/", "/api/auth-status/", "/api/status/")


def _is_exempt(path: str) -> bool:
    return any(path.startswith(p) for p in EXEMPT_PREFIXES)


def _is_pin_configured():
    try:
        from .pin_auth import is_pin_configured as _check
        return _check()
    except Exception as e:
        logger.exception("PinAuthMiddleware is_pin_configured failed: %s", e)
        return False


class PinAuthMiddleware:
    def __init__(self, get_response):
        self.get_response = get_response

    def __call__(self, request):
        try:
            path = request.path
            if path.startswith("/api/") and not _is_exempt(path):
                if not _is_pin_configured():
                    return self.get_response(request)
                if not request.session.get("pin_verified"):
                    return JsonResponse({"error": "Authentication required", "code": "pin_required"}, status=401)
        except Exception as e:
            logger.exception("PinAuthMiddleware failed: %s", e)
            return JsonResponse({"error": "Auth check failed"}, status=500)
        return self.get_response(request)
