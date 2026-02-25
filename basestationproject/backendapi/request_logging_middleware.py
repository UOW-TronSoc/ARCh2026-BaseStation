"""
Middleware that logs every request (method, path, status) so the Logs page
shows which API endpoints are being hit.
"""
import logging

logger = logging.getLogger(__name__)


class RequestLoggingMiddleware:
    """Log each request as it completes: method path status_code."""

    def __init__(self, get_response):
        self.get_response = get_response

    def __call__(self, request):
        response = self.get_response(request)
        try:
            status = getattr(response, "status_code", "?")
            msg = f"{request.method} {request.path} → {status}"
            if request.GET:
                msg += f" ?{request.GET.urlencode()}"
            logger.info(msg)
        except Exception:
            pass
        return response
