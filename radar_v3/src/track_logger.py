#!/usr/bin/env python3
"""Track log utilities for monitoring tracked object counts."""

from __future__ import annotations

import logging
from datetime import datetime
from typing import Iterable, Optional


class TrackLogger:
    """Monitor tracked object counts and emit logs when the total changes."""

    def __init__(self, logger: Optional[logging.Logger] = None, *, enabled: bool = True) -> None:
        self._logger = logger or logging.getLogger(__name__)
        self._enabled = enabled
        self._last_count: Optional[int] = None

    def handle_objects(self, objects: Optional[Iterable[object]], timestamp: float) -> None:
        """Check the tracked object list and log when the count changes."""
        if not self._enabled:
            return

        if objects is None:
            current_count = 0
        elif isinstance(objects, (list, tuple, set)):
            current_count = len(objects)
        else:
            current_count = len(list(objects))
        if self._last_count == current_count:
            return

        previous = self._last_count if self._last_count is not None else 0
        delta = current_count - previous
        direction = "increased" if delta > 0 else "decreased"
        human_time = datetime.fromtimestamp(timestamp).isoformat(timespec="seconds")

        if self._last_count is None:
            message = (
                "Tracked object count initialised to %d at %s"
                % (current_count, human_time)
            )
        else:
            message = (
                "Tracked object count %s from %d to %d (%+d) at %s"
                % (direction, previous, current_count, delta, human_time)
            )

        self._logger.info(message)
        self._last_count = current_count

    def reset(self) -> None:
        """Clear internal state (e.g., when a session restarts)."""
        self._last_count = None
