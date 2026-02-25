import logging
from django.apps import AppConfig

from .log_buffer import LogBufferHandler


class BackendapiConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'backendapi'

    def ready(self):
        # Capture Django and backendapi logs for the Logs page (only once)
        root = logging.getLogger()
        if any(isinstance(h, LogBufferHandler) for h in root.handlers):
            return
        formatter = logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        )
        handler = LogBufferHandler()
        handler.setFormatter(formatter)
        root.addHandler(handler)
        root.setLevel(logging.INFO)
        # Startup message so the Logs page shows when the server became ready
        logging.getLogger(__name__).info(
            "Basestation Django server ready; request logging active."
        )
