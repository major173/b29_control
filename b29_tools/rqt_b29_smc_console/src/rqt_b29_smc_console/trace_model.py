from typing import List, Optional

from .models import TraceEntry


class TraceModel:
    def __init__(self, history_limit: Optional[int] = 100):
        self._history_limit = None if history_limit is None else max(0, int(history_limit))
        self._entries = []

    def update(self, message) -> TraceEntry:
        entry = TraceEntry(
            current_state=getattr(message, 'current_state', '') or '',
            previous_state=getattr(message, 'previous_state', '') or '',
            last_event=getattr(message, 'last_event', '') or '',
            transition_reason=getattr(message, 'transition_reason', '') or '',
            command_reason=getattr(message, 'command_reason', '') or '',
            output_mode=getattr(message, 'output_mode', '') or '',
        )
        self._entries.append(entry)
        if self._history_limit is not None and len(self._entries) > self._history_limit:
            del self._entries[: len(self._entries) - self._history_limit]
        return entry

    def latest(self) -> Optional[TraceEntry]:
        if not self._entries:
            return None
        return self._entries[-1]

    def history(self) -> List[TraceEntry]:
        return list(reversed(self._entries))
