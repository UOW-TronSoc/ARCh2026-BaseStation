"""
In-memory ring buffer for Django/server logs so the Logs page can display them.
"""
import logging
from collections import deque

# Keep last N log lines (format: "LEVEL timestamp message")
MAX_LINES = 2000
_lines: deque = deque(maxlen=MAX_LINES)


class LogBufferHandler(logging.Handler):
    """Append log records to a fixed-size ring buffer."""

    def emit(self, record):
        try:
            msg = self.format(record)
            _lines.append(msg)
        except Exception:
            self.handleError(record)


def get_django_log_lines():
    """Return the list of buffered log lines (newest last)."""
    return list(_lines)
