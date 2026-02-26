"""PIN verify and auth status endpoints. Used with session cookies (httpOnly)."""
import json
import logging
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_GET, require_POST
from django.views.decorators.csrf import ensure_csrf_cookie

from .pin_auth import verify_pin, is_pin_configured

logger = logging.getLogger(__name__)


@require_GET
@ensure_csrf_cookie
def auth_status_view(request):
    """
    GET /api/auth-status/
    Returns { authenticated: bool, pin_configured: bool }.
    Used to check if PIN is required and if session is authenticated.
    """
    try:
        authenticated = request.session.get("pin_verified") is True
        pin_configured = is_pin_configured()
    except Exception as e:
        logger.exception("auth_status failed: %s", e)
        return JsonResponse({"authenticated": False, "pin_configured": False})
    return JsonResponse({"authenticated": authenticated, "pin_configured": pin_configured})


@require_POST
@csrf_exempt
def pin_verify_view(request):
    """
    POST /api/pin-verify/ with { "pin": "123456" }.
    On success: sets request.session["pin_verified"] = True, returns { "ok": true }.
    Session cookie (httpOnly) is set by Django; browser caches it securely.
    """
    try:
        data = json.loads(request.body or "{}")
        pin = (data.get("pin") or "").strip()
    except json.JSONDecodeError:
        return JsonResponse({"ok": False, "error": "Invalid JSON"}, status=400)

    try:
        if not is_pin_configured():
            return JsonResponse({"ok": False, "error": "PIN not configured. Run: python manage.py set_pin 123456"}, status=503)

        if not verify_pin(pin):
            return JsonResponse({"ok": False, "error": "Invalid PIN"}, status=401)

        request.session["pin_verified"] = True
        request.session.set_expiry(60 * 60 * 24 * 30)  # 30 days
        request.session.save()
        return JsonResponse({"ok": True})
    except Exception as e:
        logger.exception("pin_verify failed: %s", e)
        return JsonResponse({"ok": False, "error": "Server error. Is Redis/session storage running?"}, status=500)
