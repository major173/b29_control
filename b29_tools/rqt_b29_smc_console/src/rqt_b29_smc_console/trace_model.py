import time
from dataclasses import replace
from typing import Callable, List, Optional

from .models import TraceEntry, TraceHistoryEntry, TraceUpdateDelta


_KEY_FIELDS = (
    'current_state',
    'last_event',
    'transition_reason',
    'command_reason',
    'output_mode',
)


class TraceModel:
    def __init__(self, history_limit: Optional[int] = 100, clock: Optional[Callable[[], float]] = None):
        self._history_limit = None if history_limit is None else max(0, int(history_limit))
        self._clock = clock or time.monotonic
        self._latest = None
        self._raw_entries = []
        self._key_entries = []
        self._last_update_delta = None

    def update(self, message) -> TraceEntry:
        entry = TraceEntry(
            current_state=getattr(message, 'current_state', '') or '',
            previous_state=getattr(message, 'previous_state', '') or '',
            last_event=getattr(message, 'last_event', '') or '',
            transition_reason=getattr(message, 'transition_reason', '') or '',
            command_reason=getattr(message, 'command_reason', '') or '',
            output_mode=getattr(message, 'output_mode', '') or '',
        )
        stamp_sec = self._message_time_sec(message)
        self._latest = entry
        raw_row = TraceHistoryEntry(
            entry=entry,
            repeat_count=1,
            duration_sec=0.0,
            first_seen_sec=stamp_sec,
            last_seen_sec=stamp_sec,
        )
        self._raw_entries.append(raw_row)
        self._trim_history(self._raw_entries)

        key_row, key_appended = self._update_key_history(entry, stamp_sec)
        self._last_update_delta = TraceUpdateDelta(
            latest=entry,
            raw_entry=raw_row,
            raw_appended=True,
            key_entry=key_row,
            key_appended=key_appended,
        )
        return entry

    def latest(self) -> Optional[TraceEntry]:
        return self._latest

    def history(self) -> List[TraceEntry]:
        return [row.entry for row in self.key_history()]

    def key_history(self) -> List[TraceHistoryEntry]:
        return list(reversed(self._key_entries))

    def raw_history(self) -> List[TraceHistoryEntry]:
        return list(reversed(self._raw_entries))

    def last_update_delta(self) -> Optional[TraceUpdateDelta]:
        return self._last_update_delta

    def clear_history(self):
        self._raw_entries = []
        self._key_entries = []
        self._last_update_delta = None

    def _update_key_history(self, entry, stamp_sec):
        if not self._key_entries:
            row = TraceHistoryEntry(
                entry=entry,
                repeat_count=1,
                duration_sec=0.0,
                first_seen_sec=stamp_sec,
                last_seen_sec=stamp_sec,
            )
            self._key_entries.append(row)
            self._trim_history(self._key_entries)
            return row, True

        previous = self._key_entries[-1]
        if self._is_key_change(previous.entry, entry):
            row = TraceHistoryEntry(
                entry=entry,
                repeat_count=1,
                duration_sec=0.0,
                first_seen_sec=stamp_sec,
                last_seen_sec=stamp_sec,
            )
            self._key_entries.append(row)
            self._trim_history(self._key_entries)
            return row, True

        row = replace(
            previous,
            entry=entry,
            repeat_count=previous.repeat_count + 1,
            duration_sec=max(0.0, stamp_sec - previous.first_seen_sec),
            last_seen_sec=stamp_sec,
        )
        self._key_entries[-1] = row
        return row, False

    @staticmethod
    def _is_key_change(previous, current):
        for field_name in _KEY_FIELDS:
            if getattr(previous, field_name) != getattr(current, field_name):
                return True
        return False

    def _message_time_sec(self, message):
        header = getattr(message, 'header', None)
        stamp = getattr(header, 'stamp', None) if header is not None else None
        if stamp is not None and hasattr(stamp, 'to_sec'):
            try:
                value = float(stamp.to_sec())
                if value > 0.0:
                    return value
            except Exception:
                pass
        return float(self._clock())

    def _trim_history(self, entries):
        if self._history_limit is not None and len(entries) > self._history_limit:
            del entries[: len(entries) - self._history_limit]
